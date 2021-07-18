using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Triangulation;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.AssemblerExtensions;
using MGroup.Solvers.DDM.Commons;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemMatrixCSparse : IFetiDPCoarseProblemMatrix
	{
		private readonly CscMatrixAssembler assembler = new CscMatrixAssembler(false, true);
		private LUCSparseNet inverseSccGlobal;
		
		public void InvertGlobalScc(int numGlobalCornerDofs, Dictionary<int, int[]> subdomainToGlobalCornerDofs, 
			Dictionary<int, IMatrix>  subdomainMatricesScc)
		{
			CscMatrix globalScc = 
				assembler.BuildGlobalMatrix(numGlobalCornerDofs, subdomainToGlobalCornerDofs, subdomainMatricesScc);
			inverseSccGlobal = LUCSparseNet.Factorize(globalScc);
		}

		public void MultiplyInverseScc(Vector input, Vector output) => inverseSccGlobal.SolveLinearSystem(input, output);

		public DofPermutation ReorderGlobalCornerDofs(int numGlobalCornerDofs, Dictionary<int, int[]> subdomainToGlobalCornerDofs)
			=> DofPermutation.CreateNoPermutation();
	}
}
