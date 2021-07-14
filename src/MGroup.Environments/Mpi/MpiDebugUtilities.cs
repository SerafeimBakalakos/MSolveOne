using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MPI;

namespace MGroup.Environments.Mpi
{
	public static class MpiDebugUtilities
	{
		[Conditional("DEBUG")]
		public static void AssistDebuggerAttachment(Intracommunicator comm = null)
		{
			if (comm == null) comm = Communicator.world;
			DoSerially(comm, () => Console.WriteLine($"MPI process {comm.Rank}: PID = {Process.GetCurrentProcess().Id}"));
			if (comm.Rank == 0)
			{
				Console.Write("All processes of the application have paused.");
				Console.Write(" While in this state you can optionally attach a debugger.");
				Console.WriteLine(" After you have finished, type anything to continue.");
				Console.ReadLine();
			}
			comm.Barrier();
		}

		public static void DoSerially(Intracommunicator comm, Action action)
		{
			comm.Barrier();
			int token = 0;
			if (comm.Rank == 0)
			{
				// Perform action in current process only
				action();

				// Send token to our right neighbor
				comm.Send(token, (comm.Rank + 1) % comm.Size, 0);

				// Receive token from our left neighbor
				comm.Receive((comm.Rank + comm.Size - 1) % comm.Size, 0, out token);
			}
			else
			{
				// Receive token from our left neighbor
				comm.Receive((comm.Rank + comm.Size - 1) % comm.Size, 0, out token);

				// Perform action in current process only
				action();

				// Pass on the token to our right neighbor
				comm.Send(token, (comm.Rank + 1) % comm.Size, 0);
			}
			comm.Barrier();
		}
	}
}
