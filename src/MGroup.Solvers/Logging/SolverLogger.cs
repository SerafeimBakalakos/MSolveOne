using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.MSolve.Solution;

//TODO: Use enums instead of strings for the solver task and dof category. Or use interfaces & enum classes, to adhere to 
//      open-closed principle.
namespace MGroup.Solvers.Logging
{
	public class SolverLogger : ISolverLogger
	{
		private readonly string solverName;
		private readonly List<(int iterations, double residualNormRatio)> iterativeAlgorithmData = new List<(int, double)>();
		private readonly List<SortedDictionary<string, long>> taskDurations = new List<SortedDictionary<string, long>>();
		private readonly List<SortedDictionary<string, int>> numDofsPerCategory = new List<SortedDictionary<string, int>>();
		private int currentStep;

		public SolverLogger(string solverName)
		{
			this.solverName = solverName;
			currentStep = 0;
			taskDurations.Add(new SortedDictionary<string, long>());
			numDofsPerCategory.Add(new SortedDictionary<string, int>());
		}

		public string ExtraInfo { get; set; }


		public int GetNumDofs(int analysisStep, string category) => numDofsPerCategory[analysisStep][category];

		public int GetNumIterationsOfIterativeAlgorithm(int analysisStep) => iterativeAlgorithmData[analysisStep].iterations;
		public double GetResidualNormRatioOfIterativeAlgorithm(int analysisStep) 
			=> iterativeAlgorithmData[analysisStep].residualNormRatio;

		/// <summary>
		/// Adds the duration of the selected task to the duration of the same task during the current analysis step.
		/// </summary>
		/// <param name="task"></param>
		/// <param name="duration"></param>
		public void LogTaskDuration(string task, long duration)
		{
			bool exists = taskDurations[currentStep].TryGetValue(task, out long durationSofar);
			taskDurations[currentStep][task] = durationSofar + duration;
		}

		public void LogNumDofs(string category, int numDofs)
			=> numDofsPerCategory[currentStep][category] = numDofs;

		public void LogIterativeAlgorithm(int iterations, double residualNormRatio)
			=> iterativeAlgorithmData.Add((iterations, residualNormRatio));

		/// <summary>
		/// Each iteration is defined by the solution phase of ISolver. Dof ordering and matrix assembly may also be included, 
		/// but they are not necessarily repeated in all analyses. Thus call it at the end of the Solve() method.
		/// </summary>
		public void IncrementAnalysisStep() //TODO: Forgetting to call this is easy. A better design is needed. 
		{
			++currentStep;
			taskDurations.Add(new SortedDictionary<string, long>());
			numDofsPerCategory.Add(new SortedDictionary<string, int>());
		}

		public void WriteToFile(string path, bool append)
		{
			if (append && !File.Exists(path)) append = false;
			using (var writer = new StreamWriter(path, append))
			{
				writer.WriteLine();
				writer.WriteLine("************************************************************************************************");
				WriteHeader(writer);

				// Dofs
				for (int i = 0; i < currentStep; ++i)
				{
					foreach (var categoryDofs in numDofsPerCategory[i])
					{
						writer.Write($"Analysis iteration {i}: ");
						writer.WriteLine($"Dof category = {categoryDofs.Key} - num dofs = {categoryDofs.Value}");
					}
				}

				// Durations
				for (int i = 0; i < currentStep; ++i)
				{
					foreach (var taskDuration in taskDurations[i])
					{
						writer.Write($"Analysis iteration {i}: ");
						writer.WriteLine($"Task = {taskDuration.Key} - duration = {taskDuration.Value} ms");
					}
				}

				// Iterative algorithm data
				if (iterativeAlgorithmData.Count == currentStep) // This is false for direct solvers
				{
					for (int i = 0; i < currentStep; ++i)
					{
						(int iter, double res) = iterativeAlgorithmData[i];
						writer.Write($"Analysis iteration {i}: ");
						writer.WriteLine($"Iterative algorithm: num iterations = {iter} - residual norm ratio = {res}");
					}
				}

				writer.WriteLine("************************************************************************************************");
				writer.WriteLine();
			}
		}

		public void WriteAggregatesToFile(string path, bool append)
		{
			if (append && !File.Exists(path)) append = false;
			using (var writer = new StreamWriter(path, append))
			{
				writer.WriteLine();
				writer.WriteLine("************************************************************************************************");
				WriteHeader(writer);

				// Dofs
				var numDofsRange = new SortedDictionary<string, (int min, int max, int total)>();
				for (int i = 0; i < currentStep; ++i)
				{
					foreach (var categoryDofs in numDofsPerCategory[i])
					{
						string category = categoryDofs.Key;
						int numDofs = categoryDofs.Value;

						bool exists = numDofsRange.TryGetValue(category, out (int min, int max, int total) rangeSoFar); 
						if (!exists)
						{
							rangeSoFar.min = int.MaxValue;
							rangeSoFar.max = int.MinValue;
							rangeSoFar.total = 0;
						}
						numDofsRange[category] =
							(
								(numDofs < rangeSoFar.min) ? numDofs : rangeSoFar.min,
								(numDofs > rangeSoFar.max) ? numDofs : rangeSoFar.max,
								rangeSoFar.total + numDofs
							);
					}
				}
				foreach (var categoryRange in numDofsRange)
				{
					string category = categoryRange.Key;
					(int min, int max, int total) = categoryRange.Value;
					double average = ((double)total) / currentStep;
					writer.WriteLine($"Dof category = {category}: min = {min} - max = {max} - average = {average}");
				}

				// Durations
				long totalDuration = 0;
				var taskTotalDurations = new SortedDictionary<string, long>();
				for (int i = 0; i < currentStep; ++i)
				{
					foreach (var taskDuration in taskDurations[i])
					{
						string task = taskDuration.Key;
						long duration = taskDuration.Value;
						totalDuration += duration;
						taskTotalDurations.TryGetValue(task, out long timeSoFar);
						taskTotalDurations[task] = timeSoFar + duration;
					}
				}
				foreach (var taskTime in taskTotalDurations)
				{
					writer.WriteLine($"Task = {taskTime.Key}: total duration = {taskTime.Value} ms");
				}
				writer.WriteLine($"All tasks: total duration = {totalDuration} ms");

				// Iterative algorithm data
				if (iterativeAlgorithmData.Count == currentStep) // This is false for direct solvers
				{
					(int iterMin, int iterMax, int iterSum, double iterAvg, double iterStdev) = 
						LoggingUtilities.CalcStatistics(iterativeAlgorithmData.Select(p => p.iterations));
					(double resNormMin, double resNormMax, _, double resNormAvg, double resNormStdev) =
						LoggingUtilities.CalcStatistics(iterativeAlgorithmData.Select(p => p.residualNormRatio));

					writer.WriteLine("Iterative algorithm iterations:" +
						$" min = {iterMin} - max = {iterMax} - sum = {iterSum} - average = {iterAvg} - stdev = {iterStdev}");
					writer.WriteLine($"Iterative algorithm residual norm ratio: " +
						$"min = {resNormMin} - max = {resNormMax} - average = {resNormAvg} - stdev = {resNormStdev}");
				}

				writer.WriteLine("************************************************************************************************");
				writer.WriteLine();
			}
		}

		private void WriteHeader(StreamWriter writer)
		{
			writer.WriteLine(DateTime.Now);
			writer.WriteLine("Solver: " + solverName);
			writer.WriteLine($"Info: {ExtraInfo.TrimEnd('\r', '\n')}");
		}
	}
}
