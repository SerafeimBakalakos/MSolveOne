using System;
using MGroup.XFEM.Tests.SpecialSolvers.HybridFries;

namespace MGroup.XFEM.Tests
{
	class Program
	{
		static void Main(string[] args)
		{
			Console.WriteLine("Hello world");

			FriesExample_7_2_1_Solvers.outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Example1\";
			FriesExample_7_2_1_Solvers.outputPlotDirectory = FriesExample_7_2_1_Solvers.outputDirectory + "plots";

			FriesExample_7_2_1_Solvers.crackMouthCoords = new double[] { 337.5, 0 };
			FriesExample_7_2_1_Solvers.crackFrontCoords = new double[] { 337.5, 75 };

			int numElementsMin = 5;
			FriesExample_7_2_1_Solvers.numElements = new int[] { 9 * numElementsMin, 2 * numElementsMin, numElementsMin };
			int numSubdomainsMin = 1;
			FriesExample_7_2_1_Solvers.numSubdomains = new int[] { 9 * numSubdomainsMin, 2 * numSubdomainsMin, numSubdomainsMin };

			FriesExample_7_2_1_Solvers.RunExampleWithDirectSolver();




			//SpecialSolvers.MpiTestSuite.RunTestsWith4Processes();
			//Multiphase.EpoxyAg.Benchmark3D.RunHomogenization();
		}
	}
}
