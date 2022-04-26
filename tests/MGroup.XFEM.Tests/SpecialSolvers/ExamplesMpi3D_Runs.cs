using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using static MGroup.XFEM.Tests.SpecialSolvers.ExamplesMpi3D;

namespace MGroup.XFEM.Tests.SpecialSolvers
{
	public class ExamplesMpi3D_Runs
	{
		public static int maxCrackSteps = 1;
		//public static int maxCrackSteps = int.MaxValue;

		public static void RunTestAnalysis()
		{
			var exampleOptions = new ExampleImpactOptions();
			exampleOptions.heavisideTol = 1E-3;
			exampleOptions.maxSteps = 3;

			var meshOptions = new MeshOptions(15, 3);

			var solverOptions = new SolverOptions(SolverChoice.PFETI_DP_I);
			solverOptions.environment = EnvironmentChoice.TPL;

			var outputOptions = new OutputOptions(false, "Test");

			RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions);
		}

		public static void RunStrongScalabilityImpact()
		{
			int minElements = 36;
			int[] numSubdomains = { 2, 3, 4, 6, 9, 12 };
			SolverChoice[] solvers = GetDdmSolvers();

			string directory = null;
			for (int r = 0; r < numSubdomains.Length; ++r)
			{
				foreach (SolverChoice solver in solvers)
				{
					var exampleOptions = new ExampleImpactOptions();
					exampleOptions.heavisideTol = 1E-3;
					exampleOptions.maxSteps = maxCrackSteps > 16 ? 16 : maxCrackSteps;

					var meshOptions = new MeshOptions(minElements, numSubdomains[r]);
					var solverOptions = new SolverOptions(solver);
					solverOptions.environment = EnvironmentChoice.TPL;

					var outputOptions = new OutputOptions(false, "Strong scalability");
					directory = outputOptions.GetOutputDirectory(exampleOptions);

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		public static void RunStrongScalability4PBB()
		{
			int minElements = 36;
			int[] numSubdomains = { 2, 3, 4, 6, 9, 12 };
			SolverChoice[] solvers = GetDdmSolvers();

			string directory = null;
			for (int r = 0; r < numSubdomains.Length; ++r)
			{
				foreach (SolverChoice solver in solvers)
				{
					var exampleOptions = new ExampleBB4POptions(337);
					exampleOptions.crackFrontY = 74;
					exampleOptions.heavisideTol = 1E-3;
					exampleOptions.maxSteps = maxCrackSteps > 13 ? 13 : maxCrackSteps;

					var meshOptions = new MeshOptions(minElements, numSubdomains[r]);
					var solverOptions = new SolverOptions(solver);
					solverOptions.environment = EnvironmentChoice.TPL;

					var outputOptions = new OutputOptions(false, "Strong scalability");
					directory = outputOptions.GetOutputDirectory(exampleOptions);

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		public static void RunWeakScalabilityImpact()
		{
			var subdomainToMeshSizeRatio = 5;
			int[] minElements = { 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55 };
			SolverChoice[] solvers = GetDdmSolvers();

			string directory = null;
			for (int r = 0; r < minElements.Length; ++r)
			{
				foreach (SolverChoice solver in solvers)
				{
					var exampleOptions = new ExampleImpactOptions();
					exampleOptions.heavisideTol = minElements[r] < 15 ? 1E-4 : 1E-3;
					exampleOptions.maxSteps = maxCrackSteps > 16 ? 16 : maxCrackSteps;

					var meshOptions = new MeshOptions(minElements[r], minElements[r] / subdomainToMeshSizeRatio);
					var solverOptions = new SolverOptions(solver);
					solverOptions.environment = EnvironmentChoice.TPL;

					var outputOptions = new OutputOptions(false, "Weak scalability");
					directory = outputOptions.GetOutputDirectory(exampleOptions);

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		public static void RunWeakScalability4PBB()
		{
			var subdomainToMeshSizeRatio = 5;
			int[] minElements = { 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
			double[] crackX = { 337.5, 337, 337.5, 336, 335, 337, 337, 337, 337, 337, 337, 337 };
			double[] heavisideTol = { 1E-4, 1E-4, 1E-4, 1E-4, 1E-4, 1E-3, 1E-3, 1E-3, 1E-3, 1E-3, 1E-3, 1E-3 };

			SolverChoice[] solvers = GetDdmSolvers();

			string directory = null;
			for (int r = 0; r < minElements.Length; ++r)
			{
				foreach (SolverChoice solver in solvers)
				{
					var exampleOptions = new ExampleBB4POptions(crackX[r]);
					exampleOptions.crackFrontY = 74;
					exampleOptions.heavisideTol = heavisideTol[r];
					exampleOptions.maxSteps = maxCrackSteps > 13 ? 13 : maxCrackSteps;

					var meshOptions = new MeshOptions(minElements[r], minElements[r] / subdomainToMeshSizeRatio);
					var solverOptions = new SolverOptions(solver);
					solverOptions.environment = EnvironmentChoice.TPL;

					var outputOptions = new OutputOptions(false, "Weak scalability");
					directory = outputOptions.GetOutputDirectory(exampleOptions);

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		private static SolverChoice[] GetDdmSolvers()
		{
			return new SolverChoice[]
			{
				SolverChoice.PFETI_DP, SolverChoice.PFETI_DP_I,
				SolverChoice.FETI_DP_D, SolverChoice.FETI_DP_D_I,
				SolverChoice.FETI_DP_L, SolverChoice.FETI_DP_L_I
			};
		}
	}
}
