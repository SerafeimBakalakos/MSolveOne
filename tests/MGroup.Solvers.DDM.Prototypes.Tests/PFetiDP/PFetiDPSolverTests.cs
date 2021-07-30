using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Constitutive.Thermal;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.Prototypes.FetiDP;
using MGroup.Solvers.DDM.Prototypes.PFetiDP;
using MGroup.Solvers.DDM.Prototypes.StrategyEnums;
using MGroup.Solvers.DDM.Tests.ExampleModels;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.Results;
using Xunit;

namespace MGroup.Solvers.DDM.Prototypes.Tests.PFetiDP
{
	public static class PFetiDPSolverTests
	{
		//TODO: Also check with homogeneous and heterogeneous scaling
		[Theory]
		[InlineData(PsmInterfaceProblem.Original, FetiDPCoarseProblem.Original, PFetiDPScaling.HomogeneousOriginal, PFetiDPPreconditioner.OriginalMonolithic)]
		[InlineData(PsmInterfaceProblem.Original, FetiDPCoarseProblem.Original, PFetiDPScaling.HomogeneousModified, PFetiDPPreconditioner.OriginalMonolithic)]
		[InlineData(PsmInterfaceProblem.Original, FetiDPCoarseProblem.Original, PFetiDPScaling.HomogeneousOriginal, PFetiDPPreconditioner.OriginalDistributive)]
		[InlineData(PsmInterfaceProblem.Distributed, FetiDPCoarseProblem.Original, PFetiDPScaling.HomogeneousDistributed, PFetiDPPreconditioner.DistributedInterfaceOriginalCoarse)]
		[InlineData(PsmInterfaceProblem.Distributed, FetiDPCoarseProblem.Distributed, PFetiDPScaling.HomogeneousDistributed, PFetiDPPreconditioner.DistributedAll)]
		[InlineData(PsmInterfaceProblem.Distributed, FetiDPCoarseProblem.DistributedJacobi, PFetiDPScaling.HomogeneousDistributed, PFetiDPPreconditioner.DistributedAll)]
		[InlineData(PsmInterfaceProblem.Distributed, FetiDPCoarseProblem.DistributedJacobiReortho, PFetiDPScaling.HomogeneousDistributed, PFetiDPPreconditioner.DistributedAll)]
		public static void TestForBrick3D(PsmInterfaceProblem interfaceProblem, FetiDPCoarseProblem coarseProblem,
			PFetiDPScaling scaling, PFetiDPPreconditioner preconditioner)
		{
			// Environment
			IComputeEnvironment environment = new SequentialSharedEnvironment();
			ComputeNodeTopology nodeTopology = Brick3DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Brick3DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures();
			ICornerDofSelection cornerDofs = Brick3DExample.GetCornerDofs(model);

			// Solver
			var solverFactory = new PFetiDPSolver.Factory(
				true, 1E-10, 200, interfaceProblem, coarseProblem, scaling, preconditioner);
			DistributedAlgebraicModel<Matrix> algebraicModel = solverFactory.BuildAlgebraicModel(environment, model);
			PFetiDPSolver solver = solverFactory.BuildSolver(model, cornerDofs, algebraicModel);

			// Linear static analysis
			var problem = new ProblemThermal(model, algebraicModel, solver);
			var childAnalyzer = new LinearAnalyzer(model, algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			// Check results
			NodalResults expectedResults = Brick3DExample.GetExpectedNodalValues(model.AllDofs);
			double tolerance = 1E-7;
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				NodalResults computedResults = algebraicModel.ExtractAllResults(subdomain.ID, solver.LinearSystem.Solution);
				Assert.True(expectedResults.IsSuperSetOf(computedResults, tolerance, out string msg), msg);
			}

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 29;
			double pcgResidualNormRatioExpected = 6.641424316172292E-11;
			Assert.Equal(pcgIterationsExpected, solver.IterativeSolverStats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, solver.IterativeSolverStats.ResidualNormRatioEstimation, precision);
		}

		[Theory]
		//[InlineData(PsmInterfaceProblem.Original, FetiDPCoarseProblem.Original, PFetiDPScaling.HomogeneousOriginal, PFetiDPPreconditioner.OriginalMonolithic)]
		//[InlineData(PsmInterfaceProblem.Original, FetiDPCoarseProblem.Original, PFetiDPScaling.HomogeneousModified, PFetiDPPreconditioner.OriginalMonolithic)]
		//[InlineData(PsmInterfaceProblem.Original, FetiDPCoarseProblem.Original, PFetiDPScaling.HomogeneousOriginal, PFetiDPPreconditioner.OriginalDistributive)]
		//[InlineData(PsmInterfaceProblem.Distributed, FetiDPCoarseProblem.Original, PFetiDPScaling.HomogeneousDistributed, PFetiDPPreconditioner.DistributedInterfaceOriginalCoarse)]
		//[InlineData(PsmInterfaceProblem.Distributed, FetiDPCoarseProblem.Distributed, PFetiDPScaling.HomogeneousDistributed, PFetiDPPreconditioner.DistributedAll)]
		//[InlineData(PsmInterfaceProblem.Distributed, FetiDPCoarseProblem.DistributedJacobi, PFetiDPScaling.HomogeneousDistributed, PFetiDPPreconditioner.DistributedAll)]
		[InlineData(PsmInterfaceProblem.Distributed, FetiDPCoarseProblem.DistributedJacobiReortho, PFetiDPScaling.HomogeneousDistributed, PFetiDPPreconditioner.DistributedAll)]
		public static void TestForPlane2D(PsmInterfaceProblem interfaceProblem, FetiDPCoarseProblem coarseProblem,
			PFetiDPScaling scaling, PFetiDPPreconditioner preconditioner)
		{
			// Environment
			IComputeEnvironment environment = new SequentialSharedEnvironment();
			ComputeNodeTopology nodeTopology = Plane2DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Plane2DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures();
			ICornerDofSelection cornerDofs = Plane2DExample.GetCornerDofs(model);

			// Solver
			var solverFactory = new PFetiDPSolver.Factory(
				true, 1E-10, 200, interfaceProblem, coarseProblem, scaling, preconditioner);
			DistributedAlgebraicModel<Matrix> algebraicModel = solverFactory.BuildAlgebraicModel(environment, model);
			PFetiDPSolver solver = solverFactory.BuildSolver(model, cornerDofs, algebraicModel);

			// Linear static analysis
			var problem = new ProblemThermal(model, algebraicModel, solver);
			var childAnalyzer = new LinearAnalyzer(model, algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			// Check results
			NodalResults expectedResults = Plane2DExample.GetExpectedNodalValues(model.AllDofs);
			double tolerance = 1E-7;
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				ISubdomainFreeDofOrdering freeDofs = algebraicModel.SubdomainFreeDofOrderings[subdomain.ID];
				NodalResults computedResults = algebraicModel.ExtractAllResults(subdomain.ID, solver.LinearSystem.Solution);
				Assert.True(expectedResults.IsSuperSetOf(computedResults, tolerance, out string msg), msg);
			}

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 14;
			double pcgResidualNormRatioExpected = 2.868430313362798E-11;
			Assert.Equal(pcgIterationsExpected, solver.IterativeSolverStats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, solver.IterativeSolverStats.ResidualNormRatioEstimation, precision);
		}
	}
}
