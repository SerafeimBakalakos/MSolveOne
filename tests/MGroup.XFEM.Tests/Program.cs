using System;
using MGroup.Environments.Mpi;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Solvers.PFetiDP;
using MGroup.XFEM.Tests.Fracture.HybridFries;
using MGroup.XFEM.Tests.SpecialSolvers;
using MGroup.XFEM.Tests.SpecialSolvers.HybridFries;

namespace MGroup.XFEM.Tests
{
	class Program
	{
		static void Main(string[] args)
		{
			Console.WriteLine("Hello world");
			//Ring(args);


			//FriesExample_7_1_2.RunExample();

			//MGroup.Solvers.DDM.Tests.PFetiDP.UniformExample.Run();

			//RunExample1();
			//RunExample2();

			//ExamplesMpi3D_Runs.RunTestAnalysis();
			//ExamplesMpi3D_Runs.RunTestMpiAnalysis();
			//ExamplesMpi3D_Runs.TestReorthoPcgImpact();

			//ExamplesMpi3D_Runs.runOnCluster = false;
			//ExamplesMpi3D_Runs.maxCrackSteps = 3;
			//ExamplesMpi3D_Runs.onlyPFETI_DP_I = true;
			//ExamplesMpi3D_Runs.subdomainToMeshSizeDefault = 6;
			using (var mpiEnvironment = new MpiEnvironment())
			{
				MpiUtilities.DeclarePerProcess("is active");

				ExamplesMpi3D_Runs.InvestigateCoarseProblemsImpact(mpiEnvironment);
				ExamplesMpi3D_Runs.InvestigateCoarseProblems4PBB(mpiEnvironment);

				//ExamplesMpi3D_Runs.InvestigateCoarsePcgTolImpact(mpiEnvironment);
				//ExamplesMpi3D_Runs.InvestigateCoarsePcgTol4PBB(mpiEnvironment);

				//ExamplesMpi3D_Runs.RunWeakScalabilityImpact(mpiEnvironment);
				//ExamplesMpi3D_Runs.RunWeakScalability4PBB(mpiEnvironment);
				//ExamplesMpi3D_Runs.RunStrongScalabilityImpact(mpiEnvironment);
				//ExamplesMpi3D_Runs.RunStrongScalability4PBB(mpiEnvironment);
				//ExamplesMpi3D_Runs.RunParallelScalabilityImpact(mpiEnvironment);
				//ExamplesMpi3D_Runs.RunParallelScalability4PBB(mpiEnvironment);
			}
		}

		public static void Ring(string[] args)
		{
			MPI.Environment.Run(ref args, comm =>
			{
				string processName = $"(processor {MPI.Environment.ProcessorName}, rank {comm.Rank} / {comm.Size})";
				if (comm.Size < 2)
				{
					// Our ring needs at least two processes
					Console.WriteLine("The Ring example must be run with at least two processes.");
					Console.WriteLine("Try: mpiexec -np 4 ring.exe");
				}
				else if (comm.Rank == 0)
				{
					// Rank 0 initiates communication around the ring
					string data = "Hello world! from: " + processName;

					// Send "Hello, World!" to our right neighbor
					comm.Send(data, (comm.Rank + 1) % comm.Size, 0);

					// Receive data from our left neighbor
					comm.Receive((comm.Rank + comm.Size - 1) % comm.Size, 0, out data);

					// Add our own rank and write the results
					Console.WriteLine(data);
				}
				else
				{
					// Receive data from our left neighbor
					String data;
					comm.Receive((comm.Rank + comm.Size - 1) % comm.Size, 0, out data);

					// Add our own rank to the data
					data += ", " + processName;

					// Pass on the intermediate to our right neighbor
					comm.Send(data, (comm.Rank + 1) % comm.Size, 0);
				}
			});
		}

		private static void RunExample1()
		{
			FriesExample_7_2_1_Solvers.maxIterations = 13;
			FriesExample_7_2_1_Model.heavisideTol = 1E-4;

			//FriesExample_7_2_1_Solvers.outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Example1\";
			FriesExample_7_2_1_Solvers.outputDirectory = @"C:\Users\cluster\Desktop\Serafeim\results\Example1\";
			FriesExample_7_2_1_Solvers.outputPlotDirectory = FriesExample_7_2_1_Solvers.outputDirectory + "plots";
			FriesExample_7_2_1_Solvers.enablePlotting = false;

			FriesExample_7_2_1_Solvers.crackMouthCoords = new double[] { 337.5, 0 };
			FriesExample_7_2_1_Solvers.crackFrontCoords = new double[] { 337.5, 74 };
			//FriesExample_7_2_1_Model.v = 0.3;

			int numElementsMin = 5;
			FriesExample_7_2_1_Solvers.numElements = new int[] { 9 * numElementsMin, 2 * numElementsMin, numElementsMin };
			int numSubdomainsMin = 1;
			FriesExample_7_2_1_Solvers.numSubdomains = new int[] { 9 * numSubdomainsMin, 2 * numSubdomainsMin, numSubdomainsMin };

			FriesExample_7_2_1_Solvers.multiThreaded = true;
			FriesExample_7_2_1_Solvers.iterTol = 1E-7;
			FriesExample_7_2_1_Solvers.objectivePcgCriterion = true;
			FriesExample_7_2_1_Solvers.ddmReanalysis = true;
			FriesExample_7_2_1_Solvers.reanalysisInitialGuess = true;
			FriesExample_7_2_1_Solvers.preconditionerFetiDP = FriesExample_7_2_1_Solvers.PreconditionerFetiDP.Dirichlet;

			CrackFetiDPCornerDofs.strategy = CrackFetiDPCornerDofs.Strategy.HeavisideAndAllTipDofs;

			//FriesExample_7_2_1_Solvers.RunExampleWithPFetiDPSolver();
			FriesExample_7_2_1_Solvers.RunExampleWithFetiDPSolver();
		}

		private static void RunExample2()
		{
			FriesExample_7_2_3_Model.heavisideTol = 1E-3;
			FriesExample_7_2_3_Solvers.maxIterations = 3;

			FriesExample_7_2_3_Solvers.outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Example2\";
			//FriesExample_7_2_3_Solvers.outputDirectory = @"C:\Users\cluster\Desktop\Serafeim\results\Example2\";
			FriesExample_7_2_3_Solvers.outputPlotDirectory = FriesExample_7_2_3_Solvers.outputDirectory + "plots";
			FriesExample_7_2_3_Solvers.enablePlotting = false;

			int numElementsMin = 10;
			FriesExample_7_2_3_Solvers.numElements = new int[] { 2 * numElementsMin, numElementsMin, 2 * numElementsMin };
			int numSubdomainsMin = 2;
			FriesExample_7_2_3_Solvers.numSubdomains = new int[] { 2 * numSubdomainsMin, numSubdomainsMin, 2 * numSubdomainsMin };
			//FriesExample_7_2_3_Solvers.numElementsPerSubdomain = null;

			CrackFetiDPCornerDofs.strategy = CrackFetiDPCornerDofs.Strategy.HeavisideAndAllTipDofs;

			FriesExample_7_2_3_Solvers.multiThreaded = true;
			FriesExample_7_2_3_Solvers.iterTol = 1E-7;
			FriesExample_7_2_3_Solvers.objectivePcgCriterion = true;
			FriesExample_7_2_3_Solvers.ddmReanalysis = true;
			FriesExample_7_2_3_Solvers.reanalysisInitialGuess = true;
			FriesExample_7_2_3_Solvers.preconditionerFetiDP = FriesExample_7_2_3_Solvers.PreconditionerFetiDP.Dirichlet;

			FriesExample_7_2_3_Solvers.RunExampleWithPFetiDPSolver();
			//FriesExample_7_2_3_Solvers.RunExampleWithFetiDPSolver();
		}
	}
}
