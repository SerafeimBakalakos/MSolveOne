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
		private List<(int numIterations, double residualNormRatio)> data
			= new List<(int numIterations, double residualNormRatio)>();

		public DdmLogger(string solverName, int numSubdomains)
		{
			this.solverName = solverName;
			this.numSubdomains = numSubdomains;
		}

		public void IncrementAnalysisIteration()
		{
			++analysisIteration;
		}

		public void LogSolverConvergenceData(int numIterations, double residualNormRatio)
		{
			data.Add((numIterations, residualNormRatio));
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
			for (int t = 0; t < data.Count; ++t)
			{
				(int numIterations, double residualNorm) = data[t];
				writer.WriteLine(
					$"Analysis iteration {t}: Solver iterations = {numIterations}. Residual norm ratio = {residualNorm}");
			}
			(int min, int max, double avg, double sum) = CalcStatistics(data.Select(d => d.numIterations));
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
