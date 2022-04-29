using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Environments.Mpi
{
	public static class MpiUtilities
	{
		public static void DeclarePerProcess(string msg)
		{
			try
			{
				int rank = MPI.Communicator.world.Rank;
				int size = MPI.Communicator.world.Size;
				string fullMsg = $"Process {rank}/{size} on machine {MPI.Environment.ProcessorName}: {msg}";
				Console.WriteLine(fullMsg);
			}
			catch (Exception)
			{
				Console.WriteLine(msg);
			}
		}

		public static void DeclareRootOnlyProcess(string msg, int root = 0)
		{
			try
			{
				int rank = MPI.Communicator.world.Rank;
				int size = MPI.Communicator.world.Size;
				string fullMsg = $"Process {rank}/{size} on machine {MPI.Environment.ProcessorName}: {msg}";
				if (rank == root)
				{
					Console.WriteLine(fullMsg);
				}
			}
			catch (Exception)
			{
				Console.WriteLine(msg);
			}
		}
	}
}
