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
	public class FetiDPCoarseProblemMatrixSymmetricSuiteSparse : IFetiDPCoarseProblemMatrix
	{
		private readonly SymmetricCscMatrixAssembler assembler = new SymmetricCscMatrixAssembler(true);
		private readonly OrderingAmdSuiteSparse reordering = new OrderingAmdSuiteSparse();

		private CholeskySuiteSparse inverseSccGlobal;
		
		public void InvertGlobalScc(int numGlobalCornerDofs, Dictionary<int, int[]> subdomainToGlobalCornerDofs, 
			Dictionary<int, IMatrix>  subdomainMatricesScc)
		{
			if (inverseSccGlobal != null)
			{
				inverseSccGlobal.Dispose();
			}
			SymmetricCscMatrix globalScc = 
				assembler.BuildGlobalMatrix(numGlobalCornerDofs, subdomainToGlobalCornerDofs, subdomainMatricesScc);
			inverseSccGlobal = CholeskySuiteSparse.Factorize(globalScc, true);
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
