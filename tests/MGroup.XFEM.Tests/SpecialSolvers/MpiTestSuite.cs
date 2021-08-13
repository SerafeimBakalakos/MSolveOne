using System;
using System.Collections.Generic;
using System.Net.Http.Headers;
using System.Text;
using MGroup.Environments.Mpi;

namespace MGroup.XFEM.Tests.SpecialSolvers
{
	public static class MpiTestSuite
	{
		public static void RunTestsWith4Processes()
		{
			IMpiGlobalOperationStrategy globalOperationStrategy = new MasterSlavesGlobalOperationStrategy(0);
			//IMpiGlobalOperationStrategy globalOperationStrategy = new DemocraticGlobalOperationStrategy();
			using (var mpiEnvironment = new MpiEnvironment(globalOperationStrategy))
			{
				MpiDebugUtilities.AssistDebuggerAttachment();

				MpiDebugUtilities.DoSerially(MPI.Communicator.world,
					() => Console.WriteLine(
						$"Process {MPI.Communicator.world.Rank}: Now running PsmInterfaceProblemDofsTests.TestForLine1DInternal"));
				PlateBenchmarkSolvers.AnalyzeWithPFetiDPSolverInternal(mpiEnvironment);

				MpiDebugUtilities.DoSerially(MPI.Communicator.world,
					() => Console.WriteLine($"Process {MPI.Communicator.world.Rank}: All tests passed"));
			}
		}
	}
}
