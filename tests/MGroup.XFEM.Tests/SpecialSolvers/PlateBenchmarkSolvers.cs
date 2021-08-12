using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PFetiDP;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.Direct;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Solvers.PFetiDP;
using Xunit;

namespace MGroup.XFEM.Tests.SpecialSolvers
{
	public static class PlateBenchmarkSolvers
	{
		[Fact]
		public static void AnalyzeWithSkylineSolver()
		{
			// Model
			int[] numElements = { 48, 48 };
			XModel<IXCrackElement> model = PlateBenchmark.DescribePhysicalModel(numElements).BuildSingleSubdomainModel();
			PlateBenchmark.CreateGeometryModel(model);

			// Solver
			var factory = new SkylineSolver.Factory();
			GlobalAlgebraicModel<SkylineMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);

			PlateBenchmark.RunAnalysis(model, algebraicModel, solver);
			var crack = (ExteriorLsmCrack)model.GeometryModel.GetDiscontinuity(0);

			//DcbBenchmark.CheckCrackPropagationPath(crack);
			PlateBenchmark.WriteCrackPath(crack);
		}

		[Fact]
		public static void AnalyzeWithPFetiDPSolver()
		{
			// Model
			int[] numElements = { 48, 48 };
			int[] numSubdomains = { 8, 8 };
			int[] numClusters = { 1, 1 };
			(XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology) 
				= PlateBenchmark.DescribePhysicalModel(numElements, numSubdomains, numClusters).BuildMultiSubdomainModel();
			PlateBenchmark.CreateGeometryModel(model);

			(IAlgebraicModel algebraicModel, ISolver solver) = CreatePFetiDPSolver(model, nodeTopology);

			PlateBenchmark.RunAnalysis(model, algebraicModel, solver);
			var crack = (ExteriorLsmCrack)model.GeometryModel.GetDiscontinuity(0);

			//DcbBenchmark.CheckCrackPropagationPath(crack);
			PlateBenchmark.WriteCrackPath(crack);
		}

		public static void ScalabilityAnalysisPFetiDP()
		{
		}

		private static (IAlgebraicModel algebraicModel, ISolver solver) CreatePFetiDPSolver(
			XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
		{
			// Environment
			IComputeEnvironment environment = new SequentialSharedEnvironment();
			environment.Initialize(nodeTopology);

			// Corner dofs
			IDofType[] stdDofs = { StructuralDof.TranslationX, StructuralDof.TranslationY };
			ICornerDofSelection cornerDofs = new CrackFetiDPCornerDofs(environment, model, stdDofs,
				sub => UniformDdmCrackModelBuilder2D.FindCornerNodes(sub, 2));

			// Solver
			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(environment,
				new PsmSubdomainMatrixManagerSymmetricCSparse.Factory(),
				cornerDofs, new FetiDPSubdomainMatrixManagerSymmetricCSparse.Factory());
			solverFactory.ExplicitSubdomainMatrices = true;
			solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(
				new FetiDPCoarseProblemMatrixSymmetricCSparse());
			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = 1E-10
			};

			DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(model, algebraicModel);

			return (algebraicModel, solver);
		}
	}
}
