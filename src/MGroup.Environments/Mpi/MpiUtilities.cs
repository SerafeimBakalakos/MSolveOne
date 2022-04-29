using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Environments.Mpi
{
	public static class MpiUtilities
	{
		public static void DeclarePerProcess(string extraMsg = null)
		{
			string msg = $"Process {MPI.Communicator.world.Rank} on machine {MPI.Environment.ProcessorName}: ";
			if (extraMsg != null)
			{
				msg += extraMsg;
			}
			else
			{
				msg += "is active";
			}
			Console.WriteLine(msg);
		}
	}
}
