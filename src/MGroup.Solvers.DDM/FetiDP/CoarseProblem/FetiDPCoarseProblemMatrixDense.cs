using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.AssemblerExtensions;
using MGroup.Solvers.DDM.Commons;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemMatrixDense : IFetiDPCoarseProblemMatrix
	{
		private readonly DenseMatrixAssembler assembler = new DenseMatrixAssembler();
		private Matrix inverseSccGlobal;
		
		public void InvertGlobalScc(int numGlobalCornerDofs, Dictionary<int, int[]> subdomainToGlobalCornerDofs, 
			Dictionary<int, IMatrix>  subdomainMatricesScc)
		{
			Matrix globalScc = 
				assembler.BuildGlobalMatrix(numGlobalCornerDofs, subdomainToGlobalCornerDofs, subdomainMatricesScc);
			globalScc.InvertInPlace();
			inverseSccGlobal = globalScc;
		}

		public void MultiplyInverseScc(Vector input, Vector output) => inverseSccGlobal.MultiplyIntoResult(input, output);

		public DofPermutation ReorderGlobalCornerDofs(int numGlobalCornerDofs, Dictionary<int, int[]> subdomainToGlobalCornerDofs)
			=> DofPermutation.CreateNoPermutation();
	}
}
