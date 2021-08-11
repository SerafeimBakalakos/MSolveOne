using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Dofs;
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
	public static class DcbBenchmarkSolvers
	{
		[Fact]
		public static void AnalyzeWithPFetiDPSolver()
		{
			// Model
			int[] numElements = { 60, 21 };
			int[] numSubdomains = { 4, 3 };
			int[] numClusters = { 1, 1 };
			(XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology) 
				= DcbBenchmark.DescribePhysicalModel(numElements, numSubdomains, numClusters).BuildMultiSubdomainModel();
			DcbBenchmark.CreateGeometryModel(model);

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

			DcbBenchmark.RunAnalysis(model, algebraicModel, solver);
			var crack = (ExteriorLsmCrack)model.GeometryModel.GetDiscontinuity(0);

			//DcbBenchmark.CheckCrackPropagationPath(crack);
			DcbBenchmark.WriteCrackPath(crack);
		}

		[Fact]
		public static void AnalyzeWithSkylineSolver()
		{
			// Model
			int[] numElements = { 60, 21 };
			XModel<IXCrackElement> model = DcbBenchmark.DescribePhysicalModel(numElements).BuildSingleSubdomainModel();
			DcbBenchmark.CreateGeometryModel(model);

			// Solver
			var factory = new SkylineSolver.Factory();
			GlobalAlgebraicModel<SkylineMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);

			DcbBenchmark.RunAnalysis(model, algebraicModel, solver);
			var crack = (ExteriorLsmCrack)model.GeometryModel.GetDiscontinuity(0);

			//DcbBenchmark.CheckCrackPropagationPath(crack);
			DcbBenchmark.WriteCrackPath(crack);
		}
	}
}
