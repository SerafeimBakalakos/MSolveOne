using System;
using MGroup.XFEM.Tests.SpecialSolvers.HybridFries;

namespace MGroup.XFEM.Tests
{
	class Program
	{
		static void Main(string[] args)
		{
			Console.WriteLine("Hello world");

			//RunExample1();
			RunExample2();


			//SpecialSolvers.MpiTestSuite.RunTestsWith4Processes();
			//Multiphase.EpoxyAg.Benchmark3D.RunHomogenization();
		}

		private static void RunExample1()
		{
			//FriesExample_7_2_1_Solvers.outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Example1\";
			FriesExample_7_2_1_Solvers.outputDirectory = @"C:\Users\cluster\Desktop\Serafeim\results\Example1\";
			FriesExample_7_2_1_Solvers.outputPlotDirectory = FriesExample_7_2_1_Solvers.outputDirectory + "plots";
			FriesExample_7_2_1_Solvers.enablePlotting = false;

			FriesExample_7_2_1_Solvers.crackMouthCoords = new double[] { 337.5, 0 };
			FriesExample_7_2_1_Solvers.crackFrontCoords = new double[] { 337.5, 74 };

			int numElementsMin = 10;
			FriesExample_7_2_1_Solvers.numElements = new int[] { 9 * numElementsMin, 2 * numElementsMin, numElementsMin };
			int numSubdomainsMin = 2;
			FriesExample_7_2_1_Solvers.numSubdomains = new int[] { 9 * numSubdomainsMin, 2 * numSubdomainsMin, numSubdomainsMin };

			FriesExample_7_2_1_Solvers.multiThreaded = true;
			FriesExample_7_2_1_Solvers.reanalysis = false;
			FriesExample_7_2_1_Solvers.reanalysisExtraDofs = FriesExample_7_2_1_Solvers.ReanalysisExtraDofs.AllNearModified;
			FriesExample_7_2_1_Solvers.psmTolerance = 1E-10;

			//FriesExample_7_2_1_Solvers.RunExampleWithDirectSolver();
			//FriesExample_7_2_1_Solvers.RunExampleWithPFetiDPSolver();
			FriesExample_7_2_1_Solvers.RunExampleWithReanalysisSolver();
		}

		private static void RunExample2()
		{
			FriesExample_7_2_3_Model.heavisideTol = 1E-4;
			FriesExample_7_2_3_Solvers.maxIterations = 15;

			//FriesExample_7_2_3_Solvers.outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Example2\";
			FriesExample_7_2_3_Solvers.outputDirectory = @"C:\Users\cluster\Desktop\Serafeim\results\Example2\";
			FriesExample_7_2_3_Solvers.outputPlotDirectory = FriesExample_7_2_3_Solvers.outputDirectory + "plots";
			FriesExample_7_2_3_Solvers.enablePlotting = false;

			int numElementsMin = 10;
			FriesExample_7_2_3_Solvers.numElements = new int[] { 2 * numElementsMin, numElementsMin, 2 * numElementsMin };
			int numSubdomainsMin = 2;
			FriesExample_7_2_3_Solvers.numSubdomains = new int[] { 2 * numSubdomainsMin, numSubdomainsMin, 2 * numSubdomainsMin };
			//FriesExample_7_2_3_Solvers.numElementsPerSubdomain = null;
			FriesExample_7_2_3_Solvers.numElementsPerSubdomain = new int[][]
			{
				new int[] { 5, 5, 5, 5 },
				new int[] { 5, 5 },
				new int[] { 5, 5, 5, 5 }
			};

			FriesExample_7_2_3_Solvers.multiThreaded = true;
			FriesExample_7_2_3_Solvers.reanalysis = false;
			FriesExample_7_2_3_Solvers.reanalysisExtraDofs = FriesExample_7_2_3_Solvers.ReanalysisExtraDofs.AllNearModified;
			FriesExample_7_2_3_Solvers.psmTolerance = 1E-10;

			//FriesExample_7_2_3_Solvers.RunExampleWithDirectSolver();
			//FriesExample_7_2_3_Solvers.RunExampleWithPFetiDPSolver();
			FriesExample_7_2_3_Solvers.RunExampleWithReanalysisSolver();
		}
	}
}
