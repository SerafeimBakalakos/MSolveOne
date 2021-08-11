using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.AssemblerExtensions;
using MGroup.Solvers.DDM.Commons;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemMatrixDense : IFetiDPCoarseProblemGlobalMatrix
	{
		private readonly DenseMatrixAssembler assembler = new DenseMatrixAssembler();
		private Matrix inverseSccGlobal;

		public void Clear() { }

		public void InvertGlobalScc(int numGlobalCornerDofs, IDictionary<int, int[]> subdomainToGlobalCornerDofs, 
			IDictionary<int, IMatrix>  subdomainMatricesScc)
		{
			Matrix globalScc = 
				assembler.BuildGlobalMatrix(numGlobalCornerDofs, subdomainToGlobalCornerDofs, subdomainMatricesScc);
			globalScc.InvertInPlace();
			inverseSccGlobal = globalScc;
		}

		public void MultiplyInverseScc(Vector input, Vector output)
		{
			inverseSccGlobal.MultiplyIntoResult(input, output);
		}

		public DofPermutation ReorderGlobalCornerDofs(int numGlobalCornerDofs, 
			IDictionary<int, int[]> subdomainToGlobalCornerDofs)
			=> DofPermutation.CreateNoPermutation();
	}
}
