using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Reordering;
using MGroup.LinearAlgebra.Triangulation;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.AssemblerExtensions;
using MGroup.Solvers.DDM.Commons;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemMatrixSymmetricCSparse : IFetiDPCoarseProblemMatrix
	{
		private readonly SymmetricCscMatrixAssembler assembler = new SymmetricCscMatrixAssembler(true);
		private readonly OrderingAmdCSparseNet reordering = new OrderingAmdCSparseNet();

		private CholeskyCSparseNet inverseSccGlobal;
		
		public void InvertGlobalScc(int numGlobalCornerDofs, Dictionary<int, int[]> subdomainToGlobalCornerDofs, 
			Dictionary<int, IMatrix>  subdomainMatricesScc)
		{
			SymmetricCscMatrix globalScc = 
				assembler.BuildGlobalMatrix(numGlobalCornerDofs, subdomainToGlobalCornerDofs, subdomainMatricesScc);
			inverseSccGlobal = CholeskyCSparseNet.Factorize(globalScc);
		}

		public void MultiplyInverseScc(Vector input, Vector output) => inverseSccGlobal.SolveLinearSystem(input, output);

		public DofPermutation ReorderGlobalCornerDofs(int numGlobalCornerDofs, Dictionary<int, int[]> subdomainToGlobalCornerDofs)
		{
			var pattern = SparsityPatternSymmetric.CreateEmpty(numGlobalCornerDofs);
			foreach (int s in subdomainToGlobalCornerDofs.Keys)
			{
				int[] globalDofs = subdomainToGlobalCornerDofs[s];
				pattern.ConnectIndices(globalDofs, false);
			}
			(int[] permutation, bool oldToNew) = reordering.FindPermutation(pattern);
			return DofPermutation.Create(permutation, oldToNew);
		}
	}
}
