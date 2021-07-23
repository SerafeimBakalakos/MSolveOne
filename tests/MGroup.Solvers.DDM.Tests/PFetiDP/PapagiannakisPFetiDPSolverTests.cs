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
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PFetiDP;
using MGroup.Solvers.DDM.Psm;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.DDM.Tests.ExampleModels;
using MGroup.Solvers.DofOrdering;
using Xunit;

namespace MGroup.Solvers.DDM.Tests.PFetiDP
{
	public static class PapagiannakisPFetiDPSolverTests
	{
		[Theory]
		[InlineData(1.0, 11, EnvironmentChoice.SequentialSharedEnvironment)]
		[InlineData(1E2, 11, EnvironmentChoice.SequentialSharedEnvironment)]
		[InlineData(1E3, 12, EnvironmentChoice.SequentialSharedEnvironment)]
		[InlineData(1E4, 12, EnvironmentChoice.SequentialSharedEnvironment)]
		[InlineData(1E5, 12, EnvironmentChoice.SequentialSharedEnvironment)]
		[InlineData(1E6, 13, EnvironmentChoice.SequentialSharedEnvironment)]
		public static void RunTest_9_1(double stiffnessRatio, int numIterationsExpected, EnvironmentChoice environmentChoice)
			=> RunTest_9_1_Internal(stiffnessRatio, numIterationsExpected, Utilities.CreateEnvironment(environmentChoice));

		internal static void RunTest_9_1_Internal(double stiffnessRatio, int numIterationsExpected,
			IComputeEnvironment environment, bool isCoarseProblemDistributed = false)
		{
			// Model
			(Model model, ComputeNodeTopology nodeTopology) = PapagiannakisModel_9_1.CreateMultiSubdomainModel(stiffnessRatio);
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			ICornerDofSelection cornerDofs = PapagiannakisModel_9_1.GetCornerDofs(model);

			// Environment
			environment.Initialize(nodeTopology);

			// Solver
			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, new PsmSubdomainMatrixManagerSymmetricCSparse.Factory(),
				cornerDofs, new FetiDPSubdomainMatrixManagerSymmetricCSparse.Factory());
			var pcgBuilder = new PcgAlgorithm.Builder();
			pcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(200);
			pcgBuilder.ResidualTolerance = 1E-5;
			if (isCoarseProblemDistributed)
			{
				var coarseProblemFactory = new FetiDPCoarseProblemDistributed.Factory();
				coarseProblemFactory.CoarseProblemSolver = pcgBuilder.Build();
				solverFactory.CoarseProblemFactory = coarseProblemFactory;
			}
			else
			{
				var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCSparse();
				solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
			}
			solverFactory.InterfaceProblemSolver = pcgBuilder.Build();
			solverFactory.IsHomogeneousProblem = stiffnessRatio == 1.0;
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
			Table<int, int, double> expectedResults = PapagiannakisModel_9_1.SolveWithSkylineSolver(stiffnessRatio);
			double tolerance = 1E-4;
			environment.DoPerNode(subdomainID =>
			{
				ISubdomain subdomain = model.GetSubdomain(subdomainID);
				ISubdomainFreeDofOrdering freeDofs = algebraicModel.SubdomainFreeDofOrderings[subdomain.ID];
				Table<int, int, double> computedResults =
					Utilities.FindNodalFieldValues(subdomain, freeDofs, model, algebraicModel, solver.LinearSystem.Solution);
				Utilities.AssertSubset(expectedResults, computedResults, tolerance);
			});

			// Check convergence
			IterativeStatistics stats = solver.InterfaceProblemSolutionStats;
			Assert.InRange(stats.NumIterationsRequired, 1, numIterationsExpected);
		}
	}
}
