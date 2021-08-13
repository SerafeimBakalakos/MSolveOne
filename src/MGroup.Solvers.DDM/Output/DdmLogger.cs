using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;

namespace MGroup.Solvers.DDM.Output
{
	public class DdmLogger
	{
		private readonly string solverName;
		private readonly int numSubdomains;
		private int analysisIteration = -1;
		private SortedDictionary<int, (int numIterations, double residualNormRatio)> convergenceData
			= new SortedDictionary<int, (int numIterations, double residualNormRatio)>();

		private List<Dictionary<int, int>> problemSizeData = new List<Dictionary<int, int>>();

		public DdmLogger(string solverName, int numSubdomains)
		{
			this.solverName = solverName;
			this.numSubdomains = numSubdomains;
		}

		public void IncrementAnalysisIteration()
		{
			++analysisIteration;
			problemSizeData.Add(new Dictionary<int, int>());
		}

		/// <summary>
		/// Log the number of unique dofs for a specified level: Lvl 0 = free dofs, Lvl 1 = interface problem, 
		/// Lvl 2 = coarse problem, Lvl 3 = interface problem of coarse problem (multilevel-DDM), 
		/// Lvl 4 = coarse problem of coarse problem (multilevel-DDM), ...
		/// </summary>
		/// <param name="problemLevel"></param>
		/// <param name="size"></param>
		public void LogProblemSize(int problemLevel, int size)
		{
			problemSizeData[analysisIteration][problemLevel] = size;
		}

		public void LogSolverConvergenceData(int numIterations, double residualNormRatio)
		{
			convergenceData[analysisIteration] = (numIterations, residualNormRatio);
		}

		public void WriteToConsole()
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
		}

		public void WriteToDebug()
		{
			throw new NotImplementedException();
		}

		public void WriteToFile(string path, bool append = true)
		{
			using (var writer = new StreamWriter(path, append))
			{
#if DEBUG
				writer.AutoFlush = true; // To look at intermediate output at certain breakpoints
#endif
				WriteToStream(writer);
			}
		}

		public void WriteToStream(StreamWriter writer)
		{
			writer.WriteLine("************************************************************************************************");
			writer.WriteLine(DateTime.Now);
			writer.WriteLine($"Solver: {solverName}");
			writer.WriteLine($"Num subdomains: {numSubdomains}");
			for (int t = 0; t < convergenceData.Count; ++t)
			{
				(int numIterations, double residualNorm) = convergenceData[t];
				var msg = new StringBuilder();
				msg.Append($"Analysis iteration {t}:");
				if (problemSizeData[t].TryGetValue(0, out int size0))
				{
					msg.Append($" Global problem size = {size0}.");

				}
				if (problemSizeData[t].TryGetValue(1, out int size1))
				{
					msg.Append($" Interface problem size = {size1}.");

				}
				if (problemSizeData[t].TryGetValue(2, out int size2))
				{
					msg.Append($" Coarse problem size = {size2}.");

				}
				msg.Append($" Solver iterations = {numIterations}.");
				msg.Append($" Residual norm ratio = {residualNorm}.");
				writer.WriteLine(msg);
			}

			(int min, int max, double avg, double sum) = CalcStatistics(convergenceData.Select(d => d.Value.numIterations));
			writer.WriteLine($"Solver iteration statistics: min = {min}, max = {max}, average = {avg}, sum = {sum}");
			writer.WriteLine("************************************************************************************************");
		}

		private (int min, int max, double avg, double sum) CalcStatistics(IEnumerable<int> dataPerIteration)
		{
			int min = int.MaxValue;
			int max = int.MinValue;
			double sum = 0.0;
			int count = 0;
			foreach (int val in dataPerIteration)
			{
				if (val < min)
				{
					min = val;
				}
				if (val > max)
				{
					max = val;
				}
				sum += val;
				++count;
			}
			return (min, max, sum / count, sum);

		}
	}
}
