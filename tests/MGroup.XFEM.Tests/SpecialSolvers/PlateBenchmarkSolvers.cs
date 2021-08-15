using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.Environments;
using MGroup.Environments.Mpi;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PFetiDP;
using MGroup.Solvers.DDM.Psm;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.DDM.Tests;
using MGroup.Solvers.Direct;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Solvers.PFetiDP;
using Xunit;

namespace MGroup.XFEM.Tests.SpecialSolvers
{
	public static class PlateBenchmarkSolvers
	{
		private const string workDirectory = @"C:\Users\Serafeim\Desktop\DDM\PFETIDP\XFEM\plate2D";

		[Fact]
		public static void AnalyzeWithSkylineSolver()
		{
			// Model
			int[] numElements = { 48, 48 };
			XModel<IXCrackElement> model = PlateBenchmark.DescribePhysicalModel(numElements).BuildSingleSubdomainModel();
			PlateBenchmark.CreateGeometryModel(model);
			PlateBenchmark.SetupEnrichmentObservers(model);
			string outputDirectory = Path.Combine(workDirectory, "plots");
			PlateBenchmark.SetupModelOutput(model, outputDirectory);

			// Solver
			var factory = new SkylineSolver.Factory();
			GlobalAlgebraicModel<SkylineMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);

			PlateBenchmark.RunAnalysis(model, algebraicModel, solver);
			PlateBenchmark.WriteCrackPath(model);
		}

		[Theory]
		[InlineData(EnvironmentChoice.SequentialShared)]
		[InlineData(EnvironmentChoice.TplShared)]
		public static void AnalyzeWithPFetiDPSolver(EnvironmentChoice environmentChoice)
			=> AnalyzeWithPFetiDPSolverInternal(environmentChoice.CreateEnvironment());

		internal static void AnalyzeWithPFetiDPSolverInternal(IComputeEnvironment environment, int numClustersTotal = 1)
		{
			string outputDirectory = Path.Combine(workDirectory, "plots");

			// Model
			int[] numElements = { 48, 48 };
			int[] numSubdomains = { 4, 4 };
			int[] numClusters = GetNumClusters(numClustersTotal);
			(XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
				= PlateBenchmark.DescribePhysicalModel(numElements, numSubdomains, numClusters).BuildMultiSubdomainModel();
			PlateBenchmark.CreateGeometryModel(model);

			(IAlgebraicModel algebraicModel, PsmSolver<SymmetricCscMatrix> solver, CrackFetiDPCornerDofs cornerDofs) 
				= CreatePFetiDPSolver(environment, model, nodeTopology, outputDirectory);

			if (!(environment is MpiEnvironment))
			{
				PlateBenchmark.SetupModelOutput(model, outputDirectory);
				PlateBenchmark.SetupPartitioningOutput(
					environment, model, (CrackFetiDPCornerDofsPlusLogging)cornerDofs, outputDirectory);
			}
			
			PlateBenchmark.RunAnalysis(model, algebraicModel, solver);
			PlateBenchmark.WriteCrackPath(model);

			string path = Path.Combine(workDirectory, "pfetidp_convergence.txt");
			solver.LoggerDdm.WriteToFile(path, true);
		}

		[Fact]
		public static void StrongScalabilityAnalysisPFetiDP()
			=> StrongScalabilityAnalysisPFetiDPInternal(new SequentialSharedEnvironment());

		internal static void StrongScalabilityAnalysisPFetiDPInternal(IComputeEnvironment environment, int numClustersTotal = 1)
		{
			string path = Path.Combine(workDirectory, "pfetidp_scalability_strong.txt");

			// Const mesh, varying subdomains
			int[] numElements = { 96, 96 };
			int[] numSubdomainsPerAxis = { 2, 3, 4, 6, 8, 12, 16 };
			int[] numClusters = GetNumClusters(numClustersTotal);
			for (int r = 0; r < numSubdomainsPerAxis.Length; ++r)
			{
				int[] numSubdomains = { numSubdomainsPerAxis[r], numSubdomainsPerAxis[r] };
				(XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
					= PlateBenchmark.DescribePhysicalModel(numElements, numSubdomains, numClusters).BuildMultiSubdomainModel();
				PlateBenchmark.CreateGeometryModel(model);

				(IAlgebraicModel algebraicModel, PsmSolver<SymmetricCscMatrix> solver, _) = 
					CreatePFetiDPSolver(environment, model, nodeTopology);

				PlateBenchmark.RunAnalysis(model, algebraicModel, solver);
				//PlateBenchmark.WriteCrackPath(crack);

				solver.LoggerDdm.WriteToFile(path, true);
			}
		}

		[Fact]
		public static void WeakScalabilityAnalysisPFetiDP()
			=> WeakScalabilityAnalysisPFetiDPInternal(new SequentialSharedEnvironment());

		internal static void WeakScalabilityAnalysisPFetiDPInternal(IComputeEnvironment environment, int numClustersTotal = 1)
		{
			string path = Path.Combine(workDirectory, "pfetidp_scalability_weak.txt");

			// Const mesh, varying subdomains
			int numElementsPerSubdomain = 8;
			int[] numSubdomainsPerAxis = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
			int[] numClusters = GetNumClusters(numClustersTotal);
			for (int r = 0; r < numSubdomainsPerAxis.Length; ++r)
			{
				int numElementsPerAxis = numElementsPerSubdomain * numSubdomainsPerAxis[r];
				int[] numElements = { numElementsPerAxis, numElementsPerAxis };
				int[] numSubdomains = { numSubdomainsPerAxis[r], numSubdomainsPerAxis[r] };
				(XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
					= PlateBenchmark.DescribePhysicalModel(numElements, numSubdomains, numClusters).BuildMultiSubdomainModel();
				PlateBenchmark.CreateGeometryModel(model);

				(IAlgebraicModel algebraicModel, PsmSolver<SymmetricCscMatrix> solver, _) 
					= CreatePFetiDPSolver(environment, model, nodeTopology);

				PlateBenchmark.RunAnalysis(model, algebraicModel, solver);
				//PlateBenchmark.WriteCrackPath(crack);

				solver.LoggerDdm.WriteToFile(path, true);
			}
		}

		private static (IAlgebraicModel algebraicModel, PsmSolver<SymmetricCscMatrix> solver, CrackFetiDPCornerDofs cornerDofs)
			CreatePFetiDPSolver(IComputeEnvironment environment, XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology,
				string cornerNodesOutputDirectory = null)
		{
			// Environment
			environment.Initialize(nodeTopology);

			// Corner dofs
			IDofType[] stdDofs = { StructuralDof.TranslationX, StructuralDof.TranslationY };
			CrackFetiDPCornerDofs cornerDofs; 
			if (cornerNodesOutputDirectory != null)
			{
				cornerDofs = new CrackFetiDPCornerDofsPlusLogging(environment, model, stdDofs,
					sub => UniformDdmCrackModelBuilder2D.FindCornerNodes(sub, 2), cornerNodesOutputDirectory);
			}
			else
			{
				cornerDofs = new CrackFetiDPCornerDofs(environment, model, stdDofs,
					sub => UniformDdmCrackModelBuilder2D.FindCornerNodes(sub, 2));
			}

			// Solver
			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(environment,
				new PsmSubdomainMatrixManagerSymmetricCSparse.Factory(),
				cornerDofs, new FetiDPSubdomainMatrixManagerSymmetricCSparse.Factory());
			solverFactory.EnableLogging = true;
			solverFactory.ExplicitSubdomainMatrices = true;
			solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(
				new FetiDPCoarseProblemMatrixSymmetricCSparse());
			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = 1E-10
			};

			DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			PsmSolver<SymmetricCscMatrix> solver = solverFactory.BuildSolver(model, algebraicModel);

			return (algebraicModel, solver, cornerDofs);
		}

		private static int[] GetNumClusters(int numClustersTotal)
		{
			if (Math.Sqrt(numClustersTotal) % 1 != 0)
			{
				throw new ArgumentException("The number of clusters must be the square of some integer");
			}
			int numClustersPerAxis = (int)Math.Sqrt(numClustersTotal);
			return new int[] { numClustersPerAxis, numClustersPerAxis };
		}
	}
}
