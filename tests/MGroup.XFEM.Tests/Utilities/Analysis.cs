using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Thermal;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.NumericalAnalyzers;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Tests.Utilities
{
	public static class Analysis
	{
		//public static IMatrix RunHomogenizationAnalysisStructural2D(IXModel model,
		//    double[] minCoords, double[] maxCoords, double thickness, ISolverBuilder solverBuilder = null)
		//{
		//    Console.WriteLine("Starting homogenization analysis");

		//    if (solverBuilder == null) solverBuilder = new SuiteSparseSolver.Builder();
		//    ISolver solver = solverBuilder.BuildSolver(model);
		//    var provider = new ProblemStructural(model, solver);
		//    var rve = new StructuralSquareRve(model, minCoords, maxCoords, thickness);
		//    var homogenization = new HomogenizationAnalyzer(model, solver, provider, rve);

		//    homogenization.Initialize();
		//    homogenization.Solve();
		//    IMatrix conductivity = homogenization.MacroscopicModulus;

		//    Console.WriteLine("Analysis finished");
		//    return conductivity;
		//}

		//public static IMatrix RunHomogenizationAnalysisStructural3D(IXModel model, double[] minCoords, double[] maxCoords, 
		//    ISolverBuilder solverBuilder = null)
		//{
		//    Console.WriteLine("Starting homogenization analysis");
		//    if (solverBuilder == null) solverBuilder = new SuiteSparseSolver.Builder();
		//    ISolver solver = solverBuilder.BuildSolver(model);
		//    var provider = new ProblemStructural(model, solver);
		//    var rve = new StructuralCubicRve(model, minCoords, maxCoords);
		//    var homogenization = new HomogenizationAnalyzer(model, solver, provider, rve);

		//    homogenization.Initialize();
		//    homogenization.Solve();
		//    IMatrix conductivity = homogenization.MacroscopicModulus;

		//    Console.WriteLine("Analysis finished");
		//    return conductivity;
		//}

		//public static IMatrix RunHomogenizationAnalysisThermal2D(IXModel model, 
		//    double[] minCoords, double[] maxCoords, double thickness, bool calcMacroscopicFlux = false)
		//{
		//    Console.WriteLine("Starting homogenization analysis");

		//    var solver = (new SuiteSparseSolver.Builder()).BuildSolver(model);
		//    var provider = new ProblemThermal(model, solver);
		//    var rve = new ThermalSquareRve(model, minCoords, maxCoords, thickness);
		//    var homogenization = new HomogenizationAnalyzer(model, solver, provider, rve);
		//    if (calcMacroscopicFlux)
		//    {
		//        double[] temperatureGradient = { 200, 0 };
		//        homogenization.MacroscopicStrains = temperatureGradient;
		//    }

		//    homogenization.Initialize();
		//    homogenization.Solve();
		//    IMatrix conductivity = homogenization.MacroscopicModulus;

		//    Console.WriteLine("Analysis finished");
		//    return conductivity;
		//}

		//public static IMatrix RunHomogenizationAnalysisThermal3D(IXModel model, double[] minCoords, double[] maxCoords, 
		//    ISolverBuilder solverBuilder = null)
		//{
		//    Console.WriteLine("Starting homogenization analysis");
		//    if (solverBuilder == null) solverBuilder = new SuiteSparseSolver.Builder();
		//    ISolver solver = solverBuilder.BuildSolver(model);
		//    var provider = new ProblemThermal(model, solver);
		//    var rve = new ThermalCubicRve(model, minCoords, maxCoords);
		//    var homogenization = new HomogenizationAnalyzer(model, solver, provider, rve);

		//    homogenization.Initialize();
		//    homogenization.Solve();
		//    IMatrix conductivity = homogenization.MacroscopicModulus;

		//    Console.WriteLine("Analysis finished");
		//    return conductivity;
		//}

		public static IGlobalVector RunThermalStaticAnalysis(IXModel model, SolverChoice solverChoice = SolverChoice.Skyline)
		{
			Console.WriteLine("Starting analysis");
			(IAlgebraicModel algebraicModel, ISolver solver) = solverChoice.Create(model);
			var problem = new ProblemThermal(model, algebraicModel, solver);
			var linearAnalyzer = new LinearAnalyzer(model, algebraicModel, solver, problem);
			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, linearAnalyzer);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			Console.WriteLine("Analysis finished");
			return solver.LinearSystem.Solution;
		}

		public static IGlobalVector RunStructuralStaticAnalysis(IXModel model, SolverChoice solverChoice = SolverChoice.Skyline)
		{
			Console.WriteLine("Starting analysis");
			(IAlgebraicModel algebraicModel, ISolver solver) = solverChoice.Create(model);
			var problem = new ProblemStructural(model, algebraicModel, solver);
			var linearAnalyzer = new LinearAnalyzer(model, algebraicModel, solver, problem);
			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, linearAnalyzer);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			Console.WriteLine("Analysis finished");
			return solver.LinearSystem.Solution;
		}
	}
}
