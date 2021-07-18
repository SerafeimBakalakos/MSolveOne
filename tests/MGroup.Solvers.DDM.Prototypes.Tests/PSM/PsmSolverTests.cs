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
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.Prototypes.PSM;
using MGroup.Solvers.DDM.Prototypes.StrategyEnums;
using MGroup.Solvers.DDM.Tests.ExampleModels;
using MGroup.Solvers.DofOrdering;
using Xunit;

namespace MGroup.Solvers.DDM.Prototypes.Tests.PSM
{
	public static class PsmSolverTests
	{
		//TODO: Also check with homogeneous and heterogeneous scaling
		[Theory]
		[InlineData(PsmInterfaceProblem.Original)]
		[InlineData(PsmInterfaceProblem.Distributed)]
		public static void TestForBrick3D(PsmInterfaceProblem interfaceProblem)
		{
			// Environment
			IComputeEnvironment environment = new SequentialSharedEnvironment();
			ComputeNodeTopology nodeTopology = Brick3DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Brick3DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures();

			// Solver
			var solverFactory = new PsmSolver.Factory(true, 1E-10, 200, interfaceProblem);
			DistributedAlgebraicModel<Matrix> algebraicModel = solverFactory.BuildAlgebraicModel(environment, model);
			PsmSolver solver = solverFactory.BuildSolver(model, algebraicModel);

			// Linear static analysis
			var problem = new ProblemThermal(model, algebraicModel, solver);
			var childAnalyzer = new LinearAnalyzer(model, algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			// Check results
			Table<int, int, double> expectedResults = Brick3DExample.GetExpectedNodalValues();
			double tolerance = 1E-7;
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				ISubdomainFreeDofOrdering freeDofs = algebraicModel.SubdomainFreeDofOrderings[subdomain.ID];
				Table<int, int, double> computedResults =
					Utilities.FindNodalFieldValues(subdomain, freeDofs, algebraicModel, solver.LinearSystem.Solution);
				Utilities.AssertEqual(expectedResults, computedResults, tolerance);
			}

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 160;
			double pcgResidualNormRatioExpected = 7.487370033127084E-11;
			Assert.Equal(pcgIterationsExpected, solver.IterativeSolverStats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, solver.IterativeSolverStats.ResidualNormRatioEstimation, precision);
		}

		[Theory]
		[InlineData(PsmInterfaceProblem.Original)]
		[InlineData(PsmInterfaceProblem.Distributed)]
		public static void TestForLine1D(PsmInterfaceProblem interfaceProblem)
		{
			// Environment
			IComputeEnvironment environment = new SequentialSharedEnvironment();
			ComputeNodeTopology nodeTopology = Line1DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Line1DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures();

			// Solver
			var solverFactory = new PsmSolver.Factory(true, 1E-10, 200, interfaceProblem);
			DistributedAlgebraicModel<Matrix> algebraicModel = solverFactory.BuildAlgebraicModel(environment, model);
			PsmSolver solver = solverFactory.BuildSolver(model, algebraicModel);

			// Linear static analysis
			var problem = new ProblemThermal(model, algebraicModel, solver);
			var childAnalyzer = new LinearAnalyzer(model, algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			// Check results
			Table<int, int, double> expectedResults = Line1DExample.GetExpectedNodalValues();
			double tolerance = 1E-7;
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				ISubdomainFreeDofOrdering freeDofs = algebraicModel.SubdomainFreeDofOrderings[subdomain.ID];
				Table<int, int, double> computedResults =
					Utilities.FindNodalFieldValues(subdomain, freeDofs, algebraicModel, solver.LinearSystem.Solution);
				Utilities.AssertEqual(expectedResults, computedResults, tolerance);
			}

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 7;
			double pcgResidualNormRatioExpected = 0;
			Assert.Equal(pcgIterationsExpected, solver.IterativeSolverStats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, solver.IterativeSolverStats.ResidualNormRatioEstimation, precision);
		}

		[Theory]
		[InlineData(PsmInterfaceProblem.Original)]
		[InlineData(PsmInterfaceProblem.Distributed)]
		public static void TestForPlane2D(PsmInterfaceProblem interfaceProblem)
		{
			// Environment
			IComputeEnvironment environment = new SequentialSharedEnvironment();
			ComputeNodeTopology nodeTopology = Plane2DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Plane2DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures();

			// Solver
			var solverFactory = new PsmSolver.Factory(true, 1E-10, 200, interfaceProblem);
			DistributedAlgebraicModel<Matrix> algebraicModel = solverFactory.BuildAlgebraicModel(environment, model);
			PsmSolver solver = solverFactory.BuildSolver(model, algebraicModel);

			// Linear static analysis
			var problem = new ProblemThermal(model, algebraicModel, solver);
			var childAnalyzer = new LinearAnalyzer(model, algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			// Check results
			Table<int, int, double> expectedResults = Plane2DExample.GetExpectedNodalValues();
			double tolerance = 1E-7;
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				ISubdomainFreeDofOrdering freeDofs = algebraicModel.SubdomainFreeDofOrderings[subdomain.ID];
				Table<int, int, double> computedResults =
					Utilities.FindNodalFieldValues(subdomain, freeDofs, algebraicModel, solver.LinearSystem.Solution);
				Utilities.AssertEqual(expectedResults, computedResults, tolerance);
			}

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 63;
			double pcgResidualNormRatioExpected = 4.859075883397028E-11;
			Assert.Equal(pcgIterationsExpected, solver.IterativeSolverStats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, solver.IterativeSolverStats.ResidualNormRatioEstimation, precision);
		}
	}
}
