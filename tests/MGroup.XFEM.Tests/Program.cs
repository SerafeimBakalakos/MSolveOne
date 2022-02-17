using System;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Solvers.PFetiDP;
using MGroup.XFEM.Tests.Fracture.HybridFries;
using MGroup.XFEM.Tests.SpecialSolvers.HybridFries;

namespace MGroup.XFEM.Tests
{
	class Program
	{
		static void Main(string[] args)
		{
			Console.WriteLine("Hello world");

			//FriesExample_7_1_2.RunExample();

			//MGroup.Solvers.DDM.Tests.PFetiDP.UniformExample.Run();

			RunExample1();
			//RunExample2();


			//SpecialSolvers.MpiTestSuite.RunTestsWith4Processes();
			//Multiphase.EpoxyAg.Benchmark3D.RunHomogenization();
		}

		private static void RunExample1()
		{
			FriesExample_7_2_1_Solvers.maxIterations = 13;
			FriesExample_7_2_1_Model.heavisideTol = 1E-4;

			FriesExample_7_2_1_Solvers.outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Example1\";
			//FriesExample_7_2_1_Solvers.outputDirectory = @"C:\Users\cluster\Desktop\Serafeim\results\Example1\";
			FriesExample_7_2_1_Solvers.outputPlotDirectory = FriesExample_7_2_1_Solvers.outputDirectory + "plots";
			FriesExample_7_2_1_Solvers.enablePlotting = false;

			FriesExample_7_2_1_Solvers.crackMouthCoords = new double[] { 336, 0 };
			FriesExample_7_2_1_Solvers.crackFrontCoords = new double[] { 336, 74 };
			FriesExample_7_2_1_Model.v = 0.3;

			int numElementsMin = 20;
			FriesExample_7_2_1_Solvers.numElements = new int[] { 9 * numElementsMin, 2 * numElementsMin, numElementsMin };
			int numSubdomainsMin = 1;
			FriesExample_7_2_1_Solvers.numSubdomains = new int[] { 9 * numSubdomainsMin, 2 * numSubdomainsMin, numSubdomainsMin };

			FriesExample_7_2_1_Solvers.multiThreaded = true;
			FriesExample_7_2_1_Solvers.iterTol = 1E-7;
			FriesExample_7_2_1_Solvers.objectivePcgCriterion = false;
			FriesExample_7_2_1_Solvers.ddmReanalysis = false;
			FriesExample_7_2_1_Solvers.reanalysisExtraDofs = FriesExample_7_2_1_Solvers.ReanalysisExtraDofs.None;
			FriesExample_7_2_1_Solvers.preconditionerFetiDP = FriesExample_7_2_1_Solvers.PreconditionerFetiDP.Dirichlet;

			//CrackFetiDPCornerDofs.strategy = CrackFetiDPCornerDofs.Strategy.AllDofs;
			CrackFetiDPCornerDofs.strategy = CrackFetiDPCornerDofs.Strategy.HeavisideAndAllTipDofs;
			//CrackFetiDPCornerDofs.strategy = CrackFetiDPCornerDofs.Strategy.HeavisideAndFirstTipDofs;

			//FriesExample_7_2_1_Solvers.RunExampleWithDirectSolver();
			//FriesExample_7_2_1_Solvers.RunExampleWithReanalysisSolver();
			//FriesExample_7_2_1_Solvers.RunExampleWithReanalysisInspector();

			FriesExample_7_2_1_Solvers.RunExampleWithPcgSolver();

			//FriesExample_7_2_1_Solvers.RunExampleWithPFetiDPSolver();
			//FriesExample_7_2_1_Solvers.RunExampleWithFetiDPSolver();

			//rerun with more than 1000 iterations
		}

		private static void RunExample2()
		{
			FriesExample_7_2_3_Model.heavisideTol = 1E-4;
			FriesExample_7_2_3_Solvers.maxIterations = 16;

			FriesExample_7_2_3_Solvers.outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Example2\";
			//FriesExample_7_2_3_Solvers.outputDirectory = @"C:\Users\cluster\Desktop\Serafeim\results\Example2\";
			FriesExample_7_2_3_Solvers.outputPlotDirectory = FriesExample_7_2_3_Solvers.outputDirectory + "plots";
			FriesExample_7_2_3_Solvers.enablePlotting = false;

			int numElementsMin = 5;
			FriesExample_7_2_3_Solvers.numElements = new int[] { 2 * numElementsMin, numElementsMin, 2 * numElementsMin };
			int numSubdomainsMin = 1;
			FriesExample_7_2_3_Solvers.numSubdomains = new int[] { 2 * numSubdomainsMin, numSubdomainsMin, 2 * numSubdomainsMin };
			//FriesExample_7_2_3_Solvers.numElementsPerSubdomain = null;

			//CrackFetiDPCornerDofs.strategy = CrackFetiDPCornerDofs.Strategy.AllDofs;
			//CrackFetiDPCornerDofs.strategy = CrackFetiDPCornerDofs.Strategy.HeavisideAndAllTipDofs;
			CrackFetiDPCornerDofs.strategy = CrackFetiDPCornerDofs.Strategy.HeavisideAndFirstTipDofs;

			FriesExample_7_2_3_Solvers.multiThreaded = true;
			FriesExample_7_2_3_Solvers.iterTol = 1E-7;
			FriesExample_7_2_3_Solvers.objectivePcgCriterion = true;
			FriesExample_7_2_3_Solvers.ddmReanalysis = false;
			FriesExample_7_2_3_Solvers.reanalysisExtraDofs = FriesExample_7_2_3_Solvers.ReanalysisExtraDofs.None;
			FriesExample_7_2_3_Solvers.preconditionerFetiDP = FriesExample_7_2_3_Solvers.PreconditionerFetiDP.Dirichlet;


			//FriesExample_7_2_3_Solvers.RunExampleWithDirectSolver();
			//FriesExample_7_2_3_Solvers.RunExampleWithReanalysisSolver();
			//FriesExample_7_2_3_Solvers.RunExampleWithReanalysisInspector();

			FriesExample_7_2_3_Solvers.RunExampleWithPcgSolver();

			//FriesExample_7_2_3_Solvers.RunExampleWithPFetiDPSolver();
			//FriesExample_7_2_3_Solvers.RunExampleWithFetiDPSolver();
		}
	}
}
