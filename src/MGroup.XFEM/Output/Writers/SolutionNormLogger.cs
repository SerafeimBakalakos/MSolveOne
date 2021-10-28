using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.XFEM.Output.Writers
{
	public class SolutionNormLogger : IResultsWriter
	{
		private readonly string outputFile;
		private readonly string solverName; //TODO: This should be a generic message.
		private int iteration;

		public SolutionNormLogger(string outputFile, string solverName)
		{
			this.outputFile = outputFile;
			this.solverName = solverName;
			this.iteration = 0;
		}

		public void WriteResults(IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			double norm = solution.Norm2();
			using (var writer = new StreamWriter(outputFile, true))
			{
				writer.WriteLine($"{DateTime.Now} | {solverName}, iteration {iteration}: \t norm2(u) = {norm}");
			}

			++iteration;
		}
	}
}
