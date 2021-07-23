using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Constitutive.Thermal;
using MGroup.Environments;
using MGroup.FEM.Entities;
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
using MGroup.Solvers.Results;
using TriangleNet;
using Xunit;

namespace MGroup.Solvers.DDM.Tests.PSM
{
	public static class PapagiannakisPsmSolverTests
	{
		[Theory]
		[InlineData(1.0, 50, EnvironmentChoice.SequentialSharedEnvironment)]
		//[InlineData(1E2, 94, EnvironmentChoice.SequentialSharedEnvironment)] // In heterogeneous problems, PSM takes a lot longer to converge to the correct solution.
		//[InlineData(1E3, 120, EnvironmentChoice.SequentialSharedEnvironment)]
		//[InlineData(1E4, 163, EnvironmentChoice.SequentialSharedEnvironment)]
		//[InlineData(1E5, 192, EnvironmentChoice.SequentialSharedEnvironment)]
		//[InlineData(1E6, 238, EnvironmentChoice.SequentialSharedEnvironment)]
		public static void RunTest_9_1(double stiffnessRatio, int numIterationsExpected, EnvironmentChoice environmentChoice)
			=> RunTest_9_1_Internal(stiffnessRatio, numIterationsExpected, Utilities.CreateEnvironment(environmentChoice));

		internal static void RunTest_9_1_Internal(double stiffnessRatio, int numIterationsExpected, 
			IComputeEnvironment environment)
		{
			// Model
			(Model model, ComputeNodeTopology nodeTopology) = PapagiannakisModel_9_1.CreateMultiSubdomainModel(stiffnessRatio);
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer

			// Environment
			environment.Initialize(nodeTopology);

			// Solver
			var pcgBuilder = new PcgAlgorithm.Builder();
			pcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(1000);
			pcgBuilder.ResidualTolerance = 1E-5;
			var solverFactory = new PsmSolver<SymmetricCscMatrix>.Factory(
				environment, new PsmSubdomainMatrixManagerSymmetricCSparse.Factory());
			solverFactory.InterfaceProblemSolver = pcgBuilder.Build();
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
			NodalResults expectedResults = PapagiannakisModel_9_1.SolveWithSkylineSolver(stiffnessRatio);
			double tolerance = 1E-4;
			environment.DoPerNode(subdomainID =>
			{
				NodalResults computedResults = algebraicModel.ExtractAllResults(subdomainID, solver.LinearSystem.Solution);
				Assert.True(expectedResults.IsSuperSetOf(computedResults, tolerance, out string msg), msg);
			});

			// Check convergence
			IterativeStatistics stats = solver.InterfaceProblemSolutionStats;
			Assert.InRange(stats.NumIterationsRequired, 1, numIterationsExpected);
		}
	}
}
