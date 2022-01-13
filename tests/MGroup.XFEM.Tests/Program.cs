using System;
using MGroup.XFEM.Tests.SpecialSolvers.HybridFries;

namespace MGroup.XFEM.Tests
{
	class Program
	{
		static void Main(string[] args)
		{
			Console.WriteLine("Hello world");

			//MGroup.Solvers.DDM.Tests.PFetiDP.UniformExample.Run();

			RunExample1();
			//RunExample2();


			//SpecialSolvers.MpiTestSuite.RunTestsWith4Processes();
			//Multiphase.EpoxyAg.Benchmark3D.RunHomogenization();
		}

		private static void RunExample1()
		{
			FriesExample_7_2_1_Solvers.maxIterations = 5;
			FriesExample_7_2_1_Model.heavisideTol = 1E-4;

			FriesExample_7_2_1_Solvers.outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Example1\";
			//FriesExample_7_2_1_Solvers.outputDirectory = @"C:\Users\cluster\Desktop\Serafeim\results\Example1\";
			FriesExample_7_2_1_Solvers.outputPlotDirectory = FriesExample_7_2_1_Solvers.outputDirectory + "plots";
			FriesExample_7_2_1_Solvers.enablePlotting = false;

			FriesExample_7_2_1_Solvers.crackMouthCoords = new double[] { 335, 0 };
			FriesExample_7_2_1_Solvers.crackFrontCoords = new double[] { 335, 74 };

			int numElementsMin = 25;
			FriesExample_7_2_1_Solvers.numElements = new int[] { 9 * numElementsMin, 2 * numElementsMin, numElementsMin };
			int numSubdomainsMin = 5;
			FriesExample_7_2_1_Solvers.numSubdomains = new int[] { 9 * numSubdomainsMin, 2 * numSubdomainsMin, numSubdomainsMin };

			FriesExample_7_2_1_Solvers.multiThreaded = true;
			FriesExample_7_2_1_Solvers.iterTol = 1E-10;
			FriesExample_7_2_1_Solvers.objectivePcgCriterion = false;
			FriesExample_7_2_1_Solvers.ddmReanalysis = true;
			FriesExample_7_2_1_Solvers.reanalysisExtraDofs = FriesExample_7_2_1_Solvers.ReanalysisExtraDofs.None;

			//FriesExample_7_2_1_Solvers.RunExampleWithDirectSolver();
			//FriesExample_7_2_1_Solvers.RunExampleWithReanalysisSolver();
			//FriesExample_7_2_1_Solvers.RunExampleWithPFetiDPSolver();
			FriesExample_7_2_1_Solvers.RunExampleWithFetiDPSolver();
		}

		private static void RunExample2()
		{
			FriesExample_7_2_3_Model.heavisideTol = 1E-3;
			FriesExample_7_2_3_Solvers.maxIterations = 5;

			//FriesExample_7_2_3_Solvers.outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Example2\";
			FriesExample_7_2_3_Solvers.outputDirectory = @"C:\Users\cluster\Desktop\Serafeim\results\Example2\";
			FriesExample_7_2_3_Solvers.outputPlotDirectory = FriesExample_7_2_3_Solvers.outputDirectory + "plots";
			FriesExample_7_2_3_Solvers.enablePlotting = false;

			int numElementsMin = 35;
			FriesExample_7_2_3_Solvers.numElements = new int[] { 2 * numElementsMin, numElementsMin, 2 * numElementsMin };
			int numSubdomainsMin = 7;
			FriesExample_7_2_3_Solvers.numSubdomains = new int[] { 2 * numSubdomainsMin, numSubdomainsMin, 2 * numSubdomainsMin };
			//FriesExample_7_2_3_Solvers.numElementsPerSubdomain = null;
			FriesExample_7_2_3_Solvers.numElementsPerSubdomain = new int[][]
			{
				new int[] { 5, 5, 5, 5 },
				new int[] { 5, 5 },
				new int[] { 5, 5, 5, 5 }
			};

			FriesExample_7_2_3_Solvers.multiThreaded = true;
			FriesExample_7_2_3_Solvers.iterTol = 1E-10;
			FriesExample_7_2_3_Solvers.objectivePcgCriterion = false;
			FriesExample_7_2_3_Solvers.ddmReanalysis = false;
			FriesExample_7_2_3_Solvers.reanalysisExtraDofs = FriesExample_7_2_3_Solvers.ReanalysisExtraDofs.None;

			//FriesExample_7_2_3_Solvers.RunExampleWithDirectSolver();
			//FriesExample_7_2_3_Solvers.RunExampleWithReanalysisSolver();
			//FriesExample_7_2_3_Solvers.RunExampleWithPFetiDPSolver();
			FriesExample_7_2_3_Solvers.RunExampleWithFetiDPSolver();
		}
	}
}
