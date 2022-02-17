using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.Iterative
{
	public class ObjectiveConvergenceCriterion : IPcgResidualConvergence
	{
		private double normF0;

		public long EllapsedMilliseconds { get; set; } = 0;

		public double EstimateResidualNormRatio(PcgAlgorithmBase pcg)
		{
			var watch = new Stopwatch();

			// Find displacements at free dofs
			watch.Start();
			Vector residual = Vector.CreateZero(pcg.Residual.Length);
			pcg.Matrix.Multiply(pcg.Solution, residual);
			residual.LinearCombinationIntoThis(-1.0, pcg.Rhs, +1.0);
			double result = residual.Norm2() / normF0;
			watch.Stop();

			this.EllapsedMilliseconds += watch.ElapsedMilliseconds;

			//Console.WriteLine($"Residual norm ratio = {result}");
			return result;
		}

		public void Initialize(PcgAlgorithmBase pcg)
		{
			normF0 = pcg.Rhs.Norm2();
		}
	}
}
