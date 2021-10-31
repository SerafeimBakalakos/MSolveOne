using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.LinearSystem;

namespace MGroup.XFEM.Output.Writers
{
	public class SolutionNormLogger : IResultsWriter
	{
		private readonly string outputFile;
		private int iteration;

		public SolutionNormLogger(string outputFile)
		{
			this.outputFile = outputFile;
			this.iteration = 0;
		}

		public string ExtraInfo { get; set; }

		public void WriteResults(IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			double norm = solution.Norm2();
			
			using (var writer = new StreamWriter(outputFile, true))
			{
				if (iteration == 0)
				{
					writer.WriteLine();
					writer.WriteLine("************************************************************************************************");
					writer.WriteLine($"Info: {ExtraInfo.TrimEnd('\r', '\n')}");
				}
				writer.Write($"{DateTime.Now}, iteration {iteration}: \t norm2(u) = {norm}");
				if (solution is GlobalVector globalVector)
				{
					writer.Write($", length = {globalVector.Length}");
				}
				writer.WriteLine();
			}

			++iteration;
		}
	}
}
