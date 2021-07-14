using MGroup.Constitutive.Structural;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.Direct;

namespace MGroup.Solvers.Tests
{
	public class SolverBenchmarks
    {
		public static void SuiteSparseMemoryConsumptionDebugging()
		{
			for (int rep = 0; rep < 10; ++rep)
			{
				var benchmarkBuilder = new CantileverBeam.Builder();
				//benchmarkBuilder.Length = 5.0;
				CantileverBeam benchmark = benchmarkBuilder.BuildWithQuad4Elements(2000, 100);

				// Solver
				var solverFactory = new SuiteSparseSolver.Factory();
				var algebraicModel = solverFactory.BuildAlgebraicModel(benchmark.Model);
				using (SuiteSparseSolver solver = solverFactory.BuildSolver(algebraicModel))
				{
					// Structural problem provider
					var provider = new ProblemStructural(benchmark.Model, algebraicModel, solver);
					
					// Linear static analysis
					var childAnalyzer = new LinearAnalyzer(benchmark.Model, algebraicModel, solver, provider);
					var parentAnalyzer = new StaticAnalyzer(benchmark.Model, algebraicModel, solver, provider, childAnalyzer);

					// Run the analysis
					parentAnalyzer.Initialize();
					parentAnalyzer.Solve();
				}
			}
		}
	}
}
