using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using MGroup.Environments;
using MGroup.Environments.Mpi;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.DDM;
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PFetiDP;
using MGroup.Solvers.DDM.Psm;
using MGroup.Solvers.DDM.PSM;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.DDM.Tests;
using MGroup.Solvers.Direct;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Solvers.PaisReanalysis;
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
			string outputDirectory = Path.Combine(workDirectory, "plots");
			PlateBenchmark.SetupModelOutput(model, outputDirectory);

			// Solver
			var factory = new SkylineSolver.Factory();
			GlobalAlgebraicModel<SkylineMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);

			PlateBenchmark.RunAnalysis(model, algebraicModel, solver);
			PlateBenchmark.WriteCrackPath(model);
		}

		[Fact]
		public static void AnalyzeWithSuiteSparseSolver()
		{
			// Model
			int[] numElements = { 48, 48 };
			XModel<IXCrackElement> model = PlateBenchmark.DescribePhysicalModel(numElements).BuildSingleSubdomainModel();
			PlateBenchmark.CreateGeometryModel(model);
			string outputDirectory = Path.Combine(workDirectory, "plots");
			PlateBenchmark.SetupModelOutput(model, outputDirectory);

			// Solver
			var factory = new SuiteSparseSolver.Factory();
			GlobalAlgebraicModel<SymmetricCscMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);

			PlateBenchmark.RunAnalysis(model, algebraicModel, solver);
			PlateBenchmark.WriteCrackPath(model);
			//Debug.WriteLine($"Num dofs = {solver.LinearSystem.RhsVector.Length}");
			solver.Dispose();
		}


		[Fact]
		public static void AnalyzeWithReanalysisSolver()
		{
			// Model
			int[] numElements = { 48, 48 };
			XModel<IXCrackElement> model = PlateBenchmark.DescribePhysicalModel(numElements).BuildSingleSubdomainModel();
			PlateBenchmark.CreateGeometryModel(model);
			string outputDirectory = Path.Combine(workDirectory, "plots");
			PlateBenchmark.SetupModelOutput(model, outputDirectory);

			// Possibly enriched nodes
			var crackGeometries = new List<IImplicitCrackGeometry>();
			ExteriorLsmCrack2D fullCrack = PlateBenchmark.CreateFullCrack(model);
			crackGeometries.Add((IImplicitCrackGeometry)(fullCrack.CrackGeometry));
			double dx = (PlateBenchmark.maxCoords[0] - PlateBenchmark.minCoords[0]) / numElements[0];
			double dy = (PlateBenchmark.maxCoords[1] - PlateBenchmark.minCoords[1]) / numElements[1];
			double maxDistance = 2 * Math.Sqrt(dx * dx + dy * dy); // Enriched nodes are at most 2 elements away from the crack.
			var enrichedNodeSelector = new CrackVicinityNodeSelector(crackGeometries, maxDistance);
			//var enrichedNodeSelector = new BoundingBoxNodeSelector(new double[] { 0, 0.5 }, new double[] { 3, 2 });

			// Solver
			var dofOrderer = new ReanalysisDofOrderer(enrichedNodeSelector.CanNodeBeEnriched);
			var factory = new ReanalysisRebuildingSolver.Factory(dofOrderer);
			ReanalysisAlgebraicModel<DokMatrixAdapter> algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);

			PlateBenchmark.RunAnalysis(model, algebraicModel, solver);
			PlateBenchmark.WriteCrackPath(model);
			//Debug.WriteLine($"Num dofs = {solver.LinearSystem.RhsVector.Length}");
			solver.Dispose();
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

			(DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel, PsmSolver<SymmetricCscMatrix> solver, 
				CrackFetiDPCornerDofs cornerDofs) 
				= CreatePFetiDPSolver(environment, model, nodeTopology, true, outputDirectory);

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
					CreatePFetiDPSolver(environment, model, nodeTopology, true);

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
					= CreatePFetiDPSolver(environment, model, nodeTopology, true);

				PlateBenchmark.RunAnalysis(model, algebraicModel, solver);
				//PlateBenchmark.WriteCrackPath(crack);

				solver.LoggerDdm.WriteToFile(path, true);
			}
		}

		private static (DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel, PsmSolver<SymmetricCscMatrix> solver, 
			CrackFetiDPCornerDofs cornerDofs)
			CreatePFetiDPSolver(IComputeEnvironment environment, XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology,
				bool reanalysis, string cornerNodesOutputDirectory = null)
		{
			// Environment
			environment.Initialize(nodeTopology);

			// Corner dofs
			IDofType[] stdDofs = { StructuralDof.TranslationX, StructuralDof.TranslationY };
			CrackFetiDPCornerDofs cornerDofs; 
			if (cornerNodesOutputDirectory != null)
			{
				cornerDofs = new CrackFetiDPCornerDofsPlusLogging(environment, model, stdDofs,
					sub => UniformDdmCrackModelBuilder2D.FindCornerNodes(sub, 2));
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
			solverFactory.ExplicitSubdomainMatrices = false;
			solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(
				new FetiDPCoarseProblemMatrixSymmetricCSparse());
			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = 1E-10
			};

			if (reanalysis)
			{
				var observer = new SubdomainEnrichmentsModifiedObserver();
				model.GeometryModel.Enricher.Observers.Add(observer);
				var reanalysisOptions = PFetiDPReanalysisOptions.CreateWithAllEnabled(observer);
				//var reanalysisOptions = PFetiDPReanalysisOptions.CreateWithAllDisabled();
				reanalysisOptions.PreviousSolution = false; // This causes errors if enabled

				solverFactory.ReanalysisOptions = reanalysisOptions;
				solverFactory.SubdomainTopology = new SubdomainTopologyOptimized();
				solverFactory.ExplicitSubdomainMatrices = true;
			}

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
