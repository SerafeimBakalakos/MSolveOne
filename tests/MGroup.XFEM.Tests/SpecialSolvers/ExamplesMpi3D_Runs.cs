using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.Environments.Mpi;
using static MGroup.XFEM.Tests.SpecialSolvers.ExamplesMpi3D;

namespace MGroup.XFEM.Tests.SpecialSolvers
{
	public class ExamplesMpi3D_Runs
	{
		public static int maxCrackSteps = int.MaxValue;
		public static bool runOnCluster = true;
		public static bool onlyPFETI_DP_I = false;
		public static int subdomainToMeshSizeDefault = 5;

		public static void RunTestMpiAnalysis()
		{
			using (var mpiEnvironment = new MpiEnvironment())
			{
				//DeclareActiveProcess();
				//MpiDebugUtilities.AssistDebuggerAttachment();

				var exampleOptions = new ExampleImpactOptions();
				exampleOptions.heavisideTol = 1E-3;
				exampleOptions.maxSteps = 3;

				var meshOptions = new MeshOptions(10, 2);
				meshOptions.numClusters = new int[] { 2, 1, 2 };

				var solverOptions = new SolverOptions(SolverChoice.PFETI_DP_I);
				solverOptions.environmentChoice = EnvironmentChoice.MPI;
				solverOptions.environment = mpiEnvironment;

				var outputOptions = new OutputOptions(false, "Test MPI");

				var coarseProblemOptions = new CoarseProblemOptionsGlobal(4, 4);
				coarseProblemOptions.ForceDemocracy();

				//var coarseProblemOptions = new CoarseProblemOptionsGlobal(4, 4);
				//coarseProblemOptions.SetMasterProcess(3);

				//var coarseProblemOptions = new CoarseProblemOptionsGlobal(4, 5);
				//coarseProblemOptions.SetMasterProcess(4);

				//var coarseProblemOptions = new CoarseProblemOptionsDistributed(4, 4);
				//coarseProblemOptions.pcgTol = 1E-7;
				//coarseProblemOptions.pcgMaxIter = 1000;
				//coarseProblemOptions.reorthoPCG = true;

				RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
			}
		}

		public static void RunTestAnalysis()
		{
			var exampleOptions = new ExampleImpactOptions();
			exampleOptions.heavisideTol = 1E-3;
			exampleOptions.maxSteps = 3;

			var meshOptions = new MeshOptions(10, 2);
			meshOptions.numClusters = new int[] { 2, 1, 2 };

			var solverOptions = new SolverOptions(SolverChoice.PFETI_DP_I);
			solverOptions.environmentChoice = EnvironmentChoice.TPL;

			var outputOptions = new OutputOptions(false, "Test");

			var coarseProblemOptions = new CoarseProblemOptionsGlobal(1, 1);

			RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
		}

		//public static void TestReorthoPcgImpact()
		//{
		//	var exampleOptions = new ExampleImpactOptions();
		//	exampleOptions.heavisideTol = 1E-3;
		//	exampleOptions.maxSteps = 1;

		//	var meshOptions = new MeshOptions(10, 2);
		//	meshOptions.numClusters = new int[] { 1, 1, 1 };

		//	var solverOptions = new SolverOptions(SolverChoice.PFETI_DP_I);
		//	solverOptions.environmentChoice = EnvironmentChoice.TPL;

		//	var outputOptions = new OutputOptions(false, "Test reortho-PCG");

		//	var coarseProblemOptions = new CoarseProblemOptionsDistributed(1, 1);
		//	coarseProblemOptions.reorthoPCG = true;
		//	coarseProblemOptions.pcgTol = 1E-7;
		//	coarseProblemOptions.pcgMaxIter = 400;
		//	coarseProblemOptions.reorthoCacheSize = 100;
		//	coarseProblemOptions.reorthoCacheRetainTol = double.MaxValue;

		//	RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
		//}


		public static void InvestigateCoarseProblemsImpact(MpiEnvironment mpiEnvironment)
		{
			//MpiDebugUtilities.AssistDebuggerAttachment();

			var subdomainToMeshSizeRatio = 5;
			int[] minSubdomains = { 3, 6, 9 };
			SolverChoice[] solvers = { SolverChoice.PFETI_DP, SolverChoice.PFETI_DP_I };
			double coarsePcgTol = 1E-4;

			var coarseProblems = new List<CoarseProblemOptions>();

			var options0 = new CoarseProblemOptionsGlobal(6, mpiEnvironment.CommWorld.Size);
			options0.ForceDemocracy();
			coarseProblems.Add(options0);

			var options1 = new CoarseProblemOptionsGlobal(5, mpiEnvironment.CommWorld.Size);
			options1.SetMasterProcess(5);
			coarseProblems.Add(options1);

			//var options2 = new CoarseProblemOptionsDistributed(6, mpiEnvironment.CommWorld.Size);
			//options2.pcgMaxIter = 10000;
			//options2.pcgTol = coarsePcgTol;
			//coarseProblems.Add(options2);

			string directory = null;
			for (int i = 0; i < minSubdomains.Length; ++i)
			{
				int minElements = subdomainToMeshSizeRatio * minSubdomains[i];
				foreach (var coarseProblemOptions in coarseProblems)
				{
					int numClusters = coarseProblemOptions.numClusters;
					foreach (SolverChoice solver in solvers)
					{
						if (minElements / minSubdomains[i] <= 2)
						{
							if (solver == SolverChoice.FETI_DP_D || solver == SolverChoice.FETI_DP_D_I
								|| solver == SolverChoice.FETI_DP_L || solver == SolverChoice.FETI_DP_L_I)
							{
								continue;
							}
						}

						var exampleOptions = new ExampleImpactOptions();
						exampleOptions.heavisideTol = 1E-3;
						exampleOptions.maxSteps = maxCrackSteps > 16 ? 16 : maxCrackSteps;

						var meshOptions = new MeshOptions(minElements, minSubdomains[i]);
						meshOptions.numClusters = new int[] { 1, 1, numClusters };

						var solverOptions = new SolverOptions(solver);
						solverOptions.environmentChoice = EnvironmentChoice.MPI;
						solverOptions.environment = mpiEnvironment;

						var outputOptions = new OutputOptions(runOnCluster, "Coarse problems comparison");
						directory = outputOptions.GetOutputDirectory(exampleOptions, solverOptions);

						RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
					}
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		public static void InvestigateCoarseProblems4PBB(MpiEnvironment mpiEnvironment)
		{
			//MpiDebugUtilities.AssistDebuggerAttachment();

			var subdomainToMeshSizeRatio = 5;
			int[] minSubdomains = { 2, 4, 6 };
			SolverChoice[] solvers = { SolverChoice.PFETI_DP, SolverChoice.PFETI_DP_I };
			double coarsePcgTol = 1E-4;

			var coarseProblems = new List<CoarseProblemOptions>();

			var options0 = new CoarseProblemOptionsGlobal(6, mpiEnvironment.CommWorld.Size);
			options0.ForceDemocracy();
			coarseProblems.Add(options0);

			var options1 = new CoarseProblemOptionsGlobal(5, mpiEnvironment.CommWorld.Size);
			options1.SetMasterProcess(5);
			coarseProblems.Add(options1);

			//var options2 = new CoarseProblemOptionsDistributed(6, mpiEnvironment.CommWorld.Size);
			//options2.pcgMaxIter = 10000;
			//options2.pcgTol = coarsePcgTol;
			//coarseProblems.Add(options2);

			string directory = null;
			for (int i = 0; i < minSubdomains.Length; ++i)
			{
				int minElements = subdomainToMeshSizeRatio * minSubdomains[i];
				foreach (var coarseProblemOptions in coarseProblems)
				{
					int numClusters = coarseProblemOptions.numClusters;
					foreach (SolverChoice solver in solvers)
					{
						if (minElements / minSubdomains[i] <= 2)
						{
							if (solver == SolverChoice.FETI_DP_D || solver == SolverChoice.FETI_DP_D_I
								|| solver == SolverChoice.FETI_DP_L || solver == SolverChoice.FETI_DP_L_I)
							{
								continue;
							}
						}

						var exampleOptions = new ExampleBB4POptions(337);
						exampleOptions.crackFrontY = 74;
						exampleOptions.heavisideTol = 1E-3;
						exampleOptions.maxSteps = maxCrackSteps > 13 ? 13 : maxCrackSteps;

						var meshOptions = new MeshOptions(minElements, minSubdomains[i]);
						meshOptions.numClusters = new int[] { numClusters, 1, 1 };

						var solverOptions = new SolverOptions(solver);
						solverOptions.environmentChoice = EnvironmentChoice.MPI;
						solverOptions.environment = mpiEnvironment;

						var outputOptions = new OutputOptions(runOnCluster, "Coarse problems comparison");
						directory = outputOptions.GetOutputDirectory(exampleOptions, solverOptions);

						RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
					}
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		public static void InvestigateCoarsePcgTolImpact(MpiEnvironment mpiEnvironment)
		{
			//MpiDebugUtilities.AssistDebuggerAttachment();

			var subdomainToMeshSizeRatio = 6;
			int minSubdomains = 6;
			int minElements = subdomainToMeshSizeRatio * minSubdomains;
			int numClusters = 6;
			SolverChoice[] solvers = { SolverChoice.PFETI_DP_I };
			double[] coarsePcgTol = { /*1E-2, 1E-4,*/ 1E-6, 1E-8 };

			string directory = null;
			for (int i = 0; i < coarsePcgTol.Length; ++i)
			{
				foreach (SolverChoice solver in solvers)
				{
					if (minElements / minSubdomains <= 2)
					{
						if (solver == SolverChoice.FETI_DP_D || solver == SolverChoice.FETI_DP_D_I
							|| solver == SolverChoice.FETI_DP_L || solver == SolverChoice.FETI_DP_L_I)
						{
							continue;
						}
					}

					//MpiUtilities.DeclarePerProcess("new analysis");
					var exampleOptions = new ExampleImpactOptions();
					exampleOptions.heavisideTol = 1E-3;
					exampleOptions.maxSteps = maxCrackSteps > 16 ? 16 : maxCrackSteps;

					var meshOptions = new MeshOptions(minElements, minSubdomains);
					meshOptions.numClusters = new int[] { 1, 1, numClusters };

					var solverOptions = new SolverOptions(solver);
					solverOptions.environmentChoice = EnvironmentChoice.MPI;
					solverOptions.environment = mpiEnvironment;

					var outputOptions = new OutputOptions(runOnCluster, "Distributed coarse PCG tol");
					directory = outputOptions.GetOutputDirectory(exampleOptions, solverOptions);

					var coarseProblemOptions = new CoarseProblemOptionsDistributed(numClusters, mpiEnvironment.CommWorld.Size);
					coarseProblemOptions.pcgMaxIter = 100000;
					coarseProblemOptions.pcgTol = coarsePcgTol[i];

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}


		public static void InvestigateCoarsePcgTol4PBB(MpiEnvironment mpiEnvironment)
		{
			//MpiDebugUtilities.AssistDebuggerAttachment();

			var subdomainToMeshSizeRatio = 6;
			int minSubdomains = 4;
			int minElements = subdomainToMeshSizeRatio * minSubdomains;
			int numClusters = 6;
			SolverChoice[] solvers = { SolverChoice.PFETI_DP_I };
			double[] coarsePcgTol = { 1E-2, 1E-4, 1E-6, 1E-8 };

			string directory = null;
			for (int i = 0; i < coarsePcgTol.Length; ++i)
			{
				foreach (SolverChoice solver in solvers)
				{
					if (minElements / minSubdomains <= 2)
					{
						if (solver == SolverChoice.FETI_DP_D || solver == SolverChoice.FETI_DP_D_I
							|| solver == SolverChoice.FETI_DP_L || solver == SolverChoice.FETI_DP_L_I)
						{
							continue;
						}
					}

					//MpiUtilities.DeclarePerProcess("new analysis");
					var exampleOptions = new ExampleBB4POptions(337);
					exampleOptions.crackFrontY = 74;
					exampleOptions.heavisideTol = 1E-3;
					exampleOptions.maxSteps = maxCrackSteps > 13 ? 13 : maxCrackSteps;

					var meshOptions = new MeshOptions(minElements, minSubdomains);
					meshOptions.numClusters = new int[] { numClusters, 1, 1 };

					var solverOptions = new SolverOptions(solver);
					solverOptions.environmentChoice = EnvironmentChoice.MPI;
					solverOptions.environment = mpiEnvironment;

					var outputOptions = new OutputOptions(runOnCluster, "Distributed coarse PCG tol");
					directory = outputOptions.GetOutputDirectory(exampleOptions, solverOptions);

					var coarseProblemOptions = new CoarseProblemOptionsDistributed(numClusters, mpiEnvironment.CommWorld.Size);
					coarseProblemOptions.pcgMaxIter = 10000;
					coarseProblemOptions.pcgTol = coarsePcgTol[i];

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		public static void RunParallelScalabilityImpact(MpiEnvironment mpiEnvironment)
		{
			//MpiDebugUtilities.AssistDebuggerAttachment();

			var subdomainToMeshSizeRatio = 6;
			int minSubdomains = 6;
			int minElements = subdomainToMeshSizeRatio * minSubdomains;
			int[] numClusters = { 1, 2, 3, 4, /*6*/ };
			SolverChoice[] solvers = GetDdmSolvers();

			string directory = null;
			for (int i = 0; i < numClusters.Length; ++i)
			{
				foreach (SolverChoice solver in solvers)
				{
					if (minElements / minSubdomains <= 2)
					{
						if (solver == SolverChoice.FETI_DP_D || solver == SolverChoice.FETI_DP_D_I
							|| solver == SolverChoice.FETI_DP_L || solver == SolverChoice.FETI_DP_L_I)
						{
							continue;
						}
					}

					//MpiUtilities.DeclarePerProcess("new analysis");
					var exampleOptions = new ExampleImpactOptions();
					exampleOptions.heavisideTol = 1E-3;
					exampleOptions.maxSteps = maxCrackSteps > 16 ? 16 : maxCrackSteps;

					var meshOptions = new MeshOptions(minElements, minSubdomains);
					meshOptions.numClusters = new int[] { 1, 1, numClusters[i] };

					var solverOptions = new SolverOptions(solver);
					solverOptions.environmentChoice = EnvironmentChoice.MPI;
					solverOptions.environment = mpiEnvironment;

					var outputOptions = new OutputOptions(runOnCluster, "Parallel scalability");
					directory = outputOptions.GetOutputDirectory(exampleOptions, solverOptions);

					var coarseProblemOptions = new CoarseProblemOptionsGlobal(numClusters[i], mpiEnvironment.CommWorld.Size);
					coarseProblemOptions.ForceDemocracy();
					//var coarseProblemOptions = new CoarseProblemOptionsGlobal(1, 1);

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		public static void RunParallelScalability4PBB(MpiEnvironment mpiEnvironment)
		{
			//MpiDebugUtilities.AssistDebuggerAttachment();

			var subdomainToMeshSizeRatio = 7;
			int minSubdomains = 4;
			int minElements = subdomainToMeshSizeRatio * minSubdomains;
			int[] numClusters = { 1, 2, 3, 4, /*6*/ };
			SolverChoice[] solvers = GetDdmSolvers();

			string directory = null;
			for (int i = 0; i < numClusters.Length; ++i)
			{
				foreach (SolverChoice solver in solvers)
				{
					if (minElements / minSubdomains <= 2)
					{
						if (solver == SolverChoice.FETI_DP_D || solver == SolverChoice.FETI_DP_D_I
							|| solver == SolverChoice.FETI_DP_L || solver == SolverChoice.FETI_DP_L_I)
						{
							continue;
						}
					}

					//MpiUtilities.DeclarePerProcess("new analysis");
					var exampleOptions = new ExampleBB4POptions(337);
					exampleOptions.crackFrontY = 74;
					exampleOptions.heavisideTol = 1E-3;
					exampleOptions.maxSteps = maxCrackSteps > 13 ? 13 : maxCrackSteps;

					var meshOptions = new MeshOptions(minElements, minSubdomains);
					meshOptions.numClusters = new int[] { numClusters[i], 1, 1 };

					var solverOptions = new SolverOptions(solver);
					solverOptions.environmentChoice = EnvironmentChoice.MPI;
					solverOptions.environment = mpiEnvironment;

					var outputOptions = new OutputOptions(runOnCluster, "Parallel scalability");
					directory = outputOptions.GetOutputDirectory(exampleOptions, solverOptions);

					var coarseProblemOptions = new CoarseProblemOptionsGlobal(numClusters[i], mpiEnvironment.CommWorld.Size);
					coarseProblemOptions.ForceDemocracy();
					//var coarseProblemOptions = new CoarseProblemOptionsGlobal(1, 1);

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		public static void RunStrongScalabilityImpact(MpiEnvironment mpiEnvironment)
		{
			//MpiDebugUtilities.AssistDebuggerAttachment();

			int minElements = 36;
			int[] minSubdomains = { 3, 6, 9, 12, 18 };
			SolverChoice[] solvers = GetDdmSolvers();

			string directory = null;
			for (int i = 0; i < minSubdomains.Length; ++i)
			{
				foreach (SolverChoice solver in solvers)
				{
					if (minElements / minSubdomains[i] <= 2)
					{
						if (solver == SolverChoice.FETI_DP_D || solver == SolverChoice.FETI_DP_D_I
							|| solver == SolverChoice.FETI_DP_L || solver == SolverChoice.FETI_DP_L_I)
						{
							continue;
						}
					}

					//MpiUtilities.DeclarePerProcess("new analysis");
					var exampleOptions = new ExampleImpactOptions();
					exampleOptions.heavisideTol = 1E-3;
					exampleOptions.maxSteps = maxCrackSteps > 16 ? 16 : maxCrackSteps;

					var meshOptions = new MeshOptions(minElements, minSubdomains[i]);
					meshOptions.numClusters = new int[] { 1, 1, 6 };

					var solverOptions = new SolverOptions(solver);
					solverOptions.environmentChoice = EnvironmentChoice.MPI;
					solverOptions.environment = mpiEnvironment;

					var outputOptions = new OutputOptions(runOnCluster, "Strong scalability");
					directory = outputOptions.GetOutputDirectory(exampleOptions, solverOptions);

					var coarseProblemOptions = new CoarseProblemOptionsGlobal(6, 6);
					//coarseProblemOptions.SetMasterProcess(0);
					//var coarseProblemOptions = new CoarseProblemOptionsGlobal(1, 1);

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		public static void RunStrongScalability4PBB(MpiEnvironment mpiEnvironment)
		{
			//MpiDebugUtilities.AssistDebuggerAttachment();

			int minElements = 24;
			int[] minSubdomains = { 2, 4, 6, 8, 12 };
			SolverChoice[] solvers = GetDdmSolvers();

			string directory = null;
			for (int i = 0; i < minSubdomains.Length; ++i)
			{
				foreach (SolverChoice solver in solvers)
				{
					if (minElements / minSubdomains[i] <= 2)
					{
						if (solver == SolverChoice.FETI_DP_D || solver == SolverChoice.FETI_DP_D_I
							|| solver == SolverChoice.FETI_DP_L || solver == SolverChoice.FETI_DP_L_I)
						{
							continue;
						}
					}

					//MpiUtilities.DeclarePerProcess("new analysis");
					var exampleOptions = new ExampleBB4POptions(337);
					exampleOptions.crackFrontY = 74;
					exampleOptions.heavisideTol = 1E-3;
					exampleOptions.maxSteps = maxCrackSteps > 13 ? 13 : maxCrackSteps;

					var meshOptions = new MeshOptions(minElements, minSubdomains[i]);
					meshOptions.numClusters = new int[] { 6, 1, 1 };

					var solverOptions = new SolverOptions(solver);
					solverOptions.environmentChoice = EnvironmentChoice.MPI;
					solverOptions.environment = mpiEnvironment;

					var outputOptions = new OutputOptions(runOnCluster, "Strong scalability");
					directory = outputOptions.GetOutputDirectory(exampleOptions, solverOptions);

					var coarseProblemOptions = new CoarseProblemOptionsGlobal(6, 6);
					//coarseProblemOptions.SetMasterProcess(0);
					//var coarseProblemOptions = new CoarseProblemOptionsGlobal(1, 1);

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		public static void RunWeakScalabilityImpact(MpiEnvironment mpiEnvironment)
		{
			//MpiDebugUtilities.AssistDebuggerAttachment();

			var subdomainToMeshSizeRatio = subdomainToMeshSizeDefault;
			//int[] minSubdomains = { 3, 6, 9, 12, 15, 18, 21, 24, 27 };
			int[] minSubdomains = { 3, 6, /*9, 12*/ };

			SolverChoice[] solvers = GetDdmSolvers();

			string directory = null;
			for (int i = 0; i < minSubdomains.Length; ++i)
			{
				foreach (SolverChoice solver in solvers)
				{
					//MpiUtilities.DeclarePerProcess("new analysis");

					var exampleOptions = new ExampleImpactOptions();
					//exampleOptions.heavisideTol = minElements[r] < 15 ? 1E-4 : 1E-3;
					exampleOptions.heavisideTol = 1E-3;
					exampleOptions.maxSteps = maxCrackSteps > 16 ? 16 : maxCrackSteps;

					int minElements = minSubdomains[i] * subdomainToMeshSizeRatio;
					var meshOptions = new MeshOptions(minElements, minSubdomains[i]);
					meshOptions.numClusters = new int[] { 1, 1, 6 };


					var solverOptions = new SolverOptions(solver);
					solverOptions.environmentChoice = EnvironmentChoice.MPI;
					solverOptions.environment = mpiEnvironment;
					//solverOptions.environmentChoice = EnvironmentChoice.TPL;

					var coarseProblemOptions = new CoarseProblemOptionsGlobal(6, 6);
					//coarseProblemOptions.SetMasterProcess(0);
					//var coarseProblemOptions = new CoarseProblemOptionsGlobal(4, 1);

					var outputOptions = new OutputOptions(runOnCluster, "Weak scalability");
					directory = outputOptions.GetOutputDirectory(exampleOptions, solverOptions);

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		public static void RunWeakScalability4PBB(MpiEnvironment mpiEnvironment)
		{
			//MpiDebugUtilities.AssistDebuggerAttachment();

			var subdomainToMeshSizeRatio = subdomainToMeshSizeDefault;
			//int[] minElements = { 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
			//double[] crackX = { 337.5, 337, 337.5, 336, 335, 337, 337, 337, 337, 337, 337, 337 };
			//double[] heavisideTol = { 1E-4, 1E-4, 1E-4, 1E-4, 1E-4, 1E-3, 1E-3, 1E-3, 1E-3, 1E-3, 1E-3, 1E-3 };
			//int[] minSubdomains = { 2, 4, 6, 8, 10, 12 };
			int[] minSubdomains = { 2, 4, /*6, 8*/ };

			SolverChoice[] solvers = GetDdmSolvers();

			string directory = null;
			for (int i = 0; i < minSubdomains.Length; ++i)
			{
				foreach (SolverChoice solver in solvers)
				{
					//MpiUtilities.DeclarePerProcess("new analysis");

					var exampleOptions = new ExampleBB4POptions(337/*crackX[r]*/);
					exampleOptions.crackFrontY = 74;
					exampleOptions.heavisideTol = 1E-3/*heavisideTol[r]*/;
					exampleOptions.maxSteps = maxCrackSteps > 13 ? 13 : maxCrackSteps;

					var meshOptions = new MeshOptions(minSubdomains[i] * subdomainToMeshSizeRatio, minSubdomains[i]);
					meshOptions.numClusters = new int[] { 6, 1, 1 };

					var solverOptions = new SolverOptions(solver);
					solverOptions.environmentChoice = EnvironmentChoice.MPI;
					solverOptions.environment = mpiEnvironment;

					var outputOptions = new OutputOptions(runOnCluster, "Weak scalability");
					directory = outputOptions.GetOutputDirectory(exampleOptions, solverOptions);

					var coarseProblemOptions = new CoarseProblemOptionsGlobal(6, 6);
					//coarseProblemOptions.SetMasterProcess(0);
					//var coarseProblemOptions = new CoarseProblemOptionsGlobal(1, 1);

					RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions);
				}
			}
			File.Create(directory + "_investigation_finished.txt");
		}

		private static SolverChoice[] GetDdmSolvers()
		{
			if (onlyPFETI_DP_I)
			{
				return new SolverChoice[]
				{
					SolverChoice.PFETI_DP_I,
				};
			}
			else
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
}
