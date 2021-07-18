using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.Commons;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public interface IFetiDPCoarseProblemMatrix
	{
		void InvertGlobalScc(int numGlobalCornerDofs, Dictionary<int, int[]> subdomainToGlobalCornerDofs,
			Dictionary<int, IMatrix> subdomainMatricesScc);

		void MultiplyInverseScc(Vector input, Vector output);

		DofPermutation ReorderGlobalCornerDofs(int numGlobalCornerDofs, Dictionary<int, int[]> subdomainToGlobalCornerDofs);
	}
}
