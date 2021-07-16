using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Constitutive.Thermal;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.IterativeMethods;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.Termination;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.Psm;
using MGroup.Solvers.DDM.PSM.Dofs;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.DDM.Tests.ExampleModels;
using MGroup.Solvers.DofOrdering;
using TriangleNet;
using Xunit;

namespace MGroup.Solvers.DDM.Tests.PSM
{
	public static class PsmSolverTests
	{
		[Theory]
		[InlineData(EnvironmentChoice.SequentialSharedEnvironment)]
		[InlineData(EnvironmentChoice.TplSharedEnvironment)]
		public static void TestForBrick3D(EnvironmentChoice environmentChoice)
			=> TestForBrick3DInternal(Utilities.CreateEnvironment(environmentChoice));

		internal static void TestForBrick3DInternal(IComputeEnvironment environment)
		{
			// Environment
			ComputeNodeTopology nodeTopology = Brick3DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Brick3DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			var subdomainTopology = new SubdomainTopology(environment, model);

			// Solver
			var pcgBuilder = new PcgAlgorithm.Builder();
			pcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(200);
			pcgBuilder.ResidualTolerance = 1E-10;
			var solverFactory = new PsmSolver<SymmetricCscMatrix>.Factory(
				environment, new PsmSubdomainMatrixManagerSymmetricCSparse.Factory());
			solverFactory.InterfaceProblemSolver = pcgBuilder.Build();
			DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			PsmSolver<SymmetricCscMatrix> solver = solverFactory.BuildSolver(model, algebraicModel, subdomainTopology);

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
			environment.DoPerNode(subdomainID =>
			{
				ISubdomain subdomain = model.GetSubdomain(subdomainID);
				ISubdomainFreeDofOrdering freeDofs = algebraicModel.DofOrdering.SubdomainDofOrderings[subdomain.ID];
				Table<int, int, double> computedResults =
					Utilities.FindNodalFieldValues(subdomain, freeDofs, algebraicModel, solver.LinearSystem.Solution);
				Utilities.AssertEqual(expectedResults, computedResults, tolerance);
			});

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 160;
			double pcgResidualNormRatioExpected = 7.487370033127084E-11;
			IterativeStatistics stats = solver.InterfaceProblemSolutionStats;
			Assert.Equal(pcgIterationsExpected, stats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, stats.ResidualNormRatioEstimation, precision);
		}

		[Theory]
		[InlineData(EnvironmentChoice.SequentialSharedEnvironment)]
		[InlineData(EnvironmentChoice.TplSharedEnvironment)]
		public static void TestForLine1D(EnvironmentChoice environmentChoice)
			=> TestForLine1DInternal(Utilities.CreateEnvironment(environmentChoice));

		internal static void TestForLine1DInternal(IComputeEnvironment environment)
		{
			// Environment
			ComputeNodeTopology nodeTopology = Line1DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Line1DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			var subdomainTopology = new SubdomainTopology(environment, model);

			// Solver
			var solverFactory = new PsmSolver<SymmetricCscMatrix>.Factory(
				environment, new PsmSubdomainMatrixManagerSymmetricCSparse.Factory());
			DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			PsmSolver<SymmetricCscMatrix> solver = solverFactory.BuildSolver(model, algebraicModel, subdomainTopology);

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
			environment.DoPerNode(subdomainID =>
			{
				ISubdomain subdomain = model.GetSubdomain(subdomainID);
				ISubdomainFreeDofOrdering freeDofs = algebraicModel.DofOrdering.SubdomainDofOrderings[subdomain.ID];
				Table<int, int, double> computedResults =
					Utilities.FindNodalFieldValues(subdomain, freeDofs, algebraicModel, solver.LinearSystem.Solution);
				Utilities.AssertEqual(expectedResults, computedResults, tolerance);
			});

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 7;
			double pcgResidualNormRatioExpected = 0;
			IterativeStatistics stats = solver.InterfaceProblemSolutionStats;
			Assert.Equal(pcgIterationsExpected, stats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, stats.ResidualNormRatioEstimation, precision);
		}

		[Theory]
		[InlineData(EnvironmentChoice.SequentialSharedEnvironment)]
		[InlineData(EnvironmentChoice.TplSharedEnvironment)]
		public static void TestForPlane2D(EnvironmentChoice environmentChoice)
			=> TestForPlane2DInternal(Utilities.CreateEnvironment(environmentChoice));

		internal static void TestForPlane2DInternal(IComputeEnvironment environment)
		{
			// Environment
			ComputeNodeTopology nodeTopology = Plane2DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Plane2DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			var subdomainTopology = new SubdomainTopology(environment, model);

			// Solver
			var pcgBuilder = new PcgAlgorithm.Builder();
			pcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(200);
			pcgBuilder.ResidualTolerance = 1E-10;
			var solverFactory = new PsmSolver<SymmetricCscMatrix>.Factory(
				environment, new PsmSubdomainMatrixManagerSymmetricCSparse.Factory());
			solverFactory.InterfaceProblemSolver = pcgBuilder.Build();
			DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			PsmSolver<SymmetricCscMatrix> solver = solverFactory.BuildSolver(model, algebraicModel, subdomainTopology);

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
			environment.DoPerNode(subdomainID =>
			{
				ISubdomain subdomain = model.GetSubdomain(subdomainID);
				ISubdomainFreeDofOrdering freeDofs = algebraicModel.DofOrdering.SubdomainDofOrderings[subdomain.ID];
				Table<int, int, double> computedResults =
					Utilities.FindNodalFieldValues(subdomain, freeDofs, algebraicModel, solver.LinearSystem.Solution);
				Utilities.AssertEqual(expectedResults, computedResults, tolerance);
			});

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 63;
			double pcgResidualNormRatioExpected = 4.859075883397028E-11;
			IterativeStatistics stats = solver.InterfaceProblemSolutionStats;
			Assert.Equal(pcgIterationsExpected, stats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, stats.ResidualNormRatioEstimation, precision);
		}
	}
}
