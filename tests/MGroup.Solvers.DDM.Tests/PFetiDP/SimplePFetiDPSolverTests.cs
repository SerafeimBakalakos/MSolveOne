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
using MGroup.MSolve.Solution;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PFetiDP;
using MGroup.Solvers.DDM.Psm;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.DDM.Tests.ExampleModels;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.Results;
using Xunit;

namespace MGroup.Solvers.DDM.Tests.PFetiDP
{
	public static class SimplePFetiDPSolverTests
	{
		[Theory]
		[InlineData(EnvironmentChoice.SequentialSharedEnvironment, false)]
		[InlineData(EnvironmentChoice.SequentialSharedEnvironment, true)]
		[InlineData(EnvironmentChoice.TplSharedEnvironment, false)]
		[InlineData(EnvironmentChoice.TplSharedEnvironment, true)]
		public static void TestForBrick3D(EnvironmentChoice environmentChoice, bool isCoarseProblemDistributed)
			=> TestForBrick3DInternal(environmentChoice.CreateEnvironment(), isCoarseProblemDistributed);

		internal static void TestForBrick3DInternal(IComputeEnvironment environment, bool isCoarseProblemDistributed)
		{
			// Environment
			ComputeNodeTopology nodeTopology = Brick3DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Brick3DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			ICornerDofSelection cornerDofs = Brick3DExample.GetCornerDofs(model);

			// Solver
			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, new PsmSubdomainMatrixManagerSymmetricCSparse.Factory(),
				cornerDofs, new FetiDPSubdomainMatrixManagerSymmetricCSparse.Factory());

			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = 1E-10
			};

			if (isCoarseProblemDistributed)
			{
				var coarseProblemPcgBuilder = new PcgAlgorithm.Builder();
				coarseProblemPcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(200);
				coarseProblemPcgBuilder.ResidualTolerance = 1E-12;
				var coarseProblemFactory = new FetiDPCoarseProblemDistributed.Factory();
				coarseProblemFactory.CoarseProblemSolver = coarseProblemPcgBuilder.Build();
				solverFactory.CoarseProblemFactory = coarseProblemFactory;
			}
			else 
			{
				var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCSparse();
				solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobalShared.Factory(coarseProblemMatrix);
			}

			DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			PsmSolver<SymmetricCscMatrix> solver = solverFactory.BuildSolver(model, algebraicModel);

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
			environment.DoPerNode(subdomainID =>
			{
				NodalResults computedResults = algebraicModel.ExtractAllResults(subdomainID, solver.LinearSystem.Solution);
				Assert.True(expectedResults.IsSuperSetOf(computedResults, tolerance, out string msg), msg);
			});

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 29;
			double pcgResidualNormRatioExpected = 6.641424316172292E-11;
			IterativeStatistics stats = solver.InterfaceProblemSolutionStats;
			Assert.Equal(pcgIterationsExpected, stats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, stats.ResidualNormRatioEstimation, precision);
		}

		[Theory]
		[InlineData(EnvironmentChoice.SequentialSharedEnvironment, false)]
		[InlineData(EnvironmentChoice.SequentialSharedEnvironment, true)]
		[InlineData(EnvironmentChoice.TplSharedEnvironment, false)]
		[InlineData(EnvironmentChoice.TplSharedEnvironment, true)]
		public static void TestForPlane2D(EnvironmentChoice environmentChoice, bool isCoarseProblemDistributed)
			=> TestForPlane2DInternal(environmentChoice.CreateEnvironment(), isCoarseProblemDistributed);

		internal static void TestForPlane2DInternal(IComputeEnvironment environment, bool isCoarseProblemDistributed)
		{
			// Environment
			ComputeNodeTopology nodeTopology = Plane2DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Plane2DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			ICornerDofSelection cornerDofs = Plane2DExample.GetCornerDofs(model);

			// Solver
			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, new PsmSubdomainMatrixManagerSymmetricCSparse.Factory(),
				cornerDofs, new FetiDPSubdomainMatrixManagerSymmetricCSparse.Factory());

			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = 1E-10
			};

			if (isCoarseProblemDistributed)
			{
				var coarseProblemPcgBuilder = new PcgAlgorithm.Builder();
				coarseProblemPcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(200);
				coarseProblemPcgBuilder.ResidualTolerance = 1E-12;
				var coarseProblemFactory = new FetiDPCoarseProblemDistributed.Factory();
				coarseProblemFactory.CoarseProblemSolver = coarseProblemPcgBuilder.Build();
				solverFactory.CoarseProblemFactory = coarseProblemFactory;
			}
			else 
			{
				var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCSparse();
				solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobalMasterSlaves.Factory(coarseProblemMatrix);
			}

			DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			PsmSolver<SymmetricCscMatrix> solver = solverFactory.BuildSolver(model, algebraicModel);

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
			environment.DoPerNode(subdomainID =>
			{
				NodalResults computedResults = algebraicModel.ExtractAllResults(subdomainID, solver.LinearSystem.Solution);
				Assert.True(expectedResults.IsSuperSetOf(computedResults, tolerance, out string msg), msg);
			});

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 14;
			double pcgResidualNormRatioExpected = 2.868430313362798E-11;
			IterativeStatistics stats = solver.InterfaceProblemSolutionStats;
			Assert.Equal(pcgIterationsExpected, stats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, stats.ResidualNormRatioEstimation, precision);
		}
	}
}
