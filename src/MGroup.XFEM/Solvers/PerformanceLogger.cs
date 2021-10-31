using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.Environments;

namespace MGroup.XFEM.Solvers
{
	public class PerformanceLogger
	{
		private readonly IComputeEnvironment environment = new SequentialSharedEnvironment();
		private readonly List<long> assemblyDurations = new List<long>();
		private readonly List<long> dofOrderingDurations = new List<long>();
		private readonly List<long> solutionDurations = new List<long>();

		private int analysisIteration = -1;

		public string ExtraInfo { get; set; }

		public void IncrementAnalysisIteration()
		{
			++analysisIteration;
		}

		public void LogDofOrderingDuration(long duration)
		{
			dofOrderingDurations.Add(duration);
		}

		public void LogMatrixAssemblyDuration(long duration)
		{
			assemblyDurations.Add(duration);
		}

		public void LogSolutionDuration(long duration)
		{
			solutionDurations.Add(duration);
		}

		public void WriteToConsole()
		{
			environment.DoGlobalOperation(() =>
			{
				// Point the stream to standard output
				var writer = new StreamWriter(Console.OpenStandardOutput());
				writer.AutoFlush = true;
				Console.SetOut(writer);

				// Call the abstract method
				WriteToStream(writer);

				// Recover the standard output stream
				var standardOutput = new StreamWriter(Console.OpenStandardOutput());
				standardOutput.AutoFlush = true;
				Console.SetOut(standardOutput);
			});
		}

		public void WriteToDebug()
		{
			throw new NotImplementedException();
		}

		public void WriteToFile(string path, bool append = true)
		{
			environment.DoGlobalOperation(() =>
			{
				using (var writer = new StreamWriter(path, append))
				{
#if DEBUG
					writer.AutoFlush = true; // To look at intermediate output at certain breakpoints
#endif
					WriteToStream(writer);
				}
			});
		}

		public void WriteToStream(StreamWriter writer)
		{
			writer.WriteLine();
			writer.WriteLine("************************************************************************************************");
			writer.WriteLine(DateTime.Now);
			writer.WriteLine($"Info: {ExtraInfo.TrimEnd('\r', '\n')}");
			for (int t = 0; t < assemblyDurations.Count; ++t)
			{
				var msg = new StringBuilder();
				msg.Append($"Analysis iteration {t}:");
				msg.Append($" Dof ordering duration = {dofOrderingDurations[t]}.");
				msg.Append($" Matrix assembly duration = {assemblyDurations[t]}.");
				msg.Append($" Solution duration = {solutionDurations[t]}.");
				writer.WriteLine(msg);
			}
			writer.WriteLine($"Total dof ordering duration = {dofOrderingDurations.Sum()}");
			writer.WriteLine($"Total matrix assembly duration = {assemblyDurations.Sum()}");
			writer.WriteLine($"Total solution duration = {solutionDurations.Sum()}");
			writer.WriteLine("************************************************************************************************");
		}
	}
}
