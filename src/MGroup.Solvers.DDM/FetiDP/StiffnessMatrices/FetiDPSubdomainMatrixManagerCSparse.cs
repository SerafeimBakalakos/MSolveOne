using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Triangulation;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.LinearAlgebraExtensions;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.Assemblers;

namespace MGroup.Solvers.DDM.FetiDP.StiffnessMatrices
{
	public class FetiDPSubdomainMatrixManagerCSparse : IFetiDPSubdomainMatrixManager
	{
		private readonly SubdomainLinearSystem<CsrMatrix> linearSystem;
		private readonly FetiDPSubdomainDofs subdomainDofs;
		private readonly SubmatrixExtractorFullCsrCsc submatrixExtractor = new SubmatrixExtractorFullCsrCsc();

		private Matrix Kcc;
		private CsrMatrix Kcr;
		private CsrMatrix Krc;
		private CscMatrix Krr;
		private LUCSparseNet inverseKrr;
		private Matrix Scc;

		public FetiDPSubdomainMatrixManagerCSparse(
			SubdomainLinearSystem<CsrMatrix> linearSystem, FetiDPSubdomainDofs subdomainDofs)
		{
			this.linearSystem = linearSystem;
			this.subdomainDofs = subdomainDofs;
		}

		public bool IsEmpty => inverseKrr == null;

		public IMatrix SchurComplementOfRemainderDofs => Scc;

		public void CalcSchurComplementOfRemainderDofs()
		{
			Scc = Matrix.CreateZero(Kcc.NumRows, Kcc.NumColumns);
			SchurComplementFullCsrCsc.CalcSchurComplement(Kcc, Kcr, Krc, inverseKrr, Scc);
		}

		public void ClearSubMatrices()
		{
			inverseKrr = null;
			Kcc = null;
			Kcr = null;
			Krc = null;
			Krr = null;
			Scc = null;
		}

		public void ExtractKrrKccKrc()
		{
			int[] cornerToFree = subdomainDofs.DofsCornerToFree;
			int[] remainderToFree = subdomainDofs.DofsRemainderToFree;

			CsrMatrix Kff = linearSystem.Matrix;
			submatrixExtractor.ExtractSubmatrices(Kff, cornerToFree, remainderToFree);
			Kcc = submatrixExtractor.Submatrix00;
			Kcr = submatrixExtractor.Submatrix01;
			Krc = submatrixExtractor.Submatrix10;
			Krr = submatrixExtractor.Submatrix11;
		}

		public void HandleDofsWereModified()
		{
			ClearSubMatrices();
			submatrixExtractor.Clear();
		}

		public void InvertKrr()
		{
			inverseKrr = LUCSparseNet.Factorize(Krr);
			Krr = null; // It has not been mutated, but it is no longer needed
		}

		public Vector MultiplyInverseKrrTimes(Vector vector) => inverseKrr.SolveLinearSystem(vector);

		public Vector MultiplyKccTimes(Vector vector) => Kcc * vector;

		public Vector MultiplyKcrTimes(Vector vector) => Kcr * vector;

		public Vector MultiplyKrcTimes(Vector vector) => Krc * vector;

		public void ReorderRemainderDofs() => subdomainDofs.ReorderRemainderDofs(DofPermutation.CreateNoPermutation());

		public class Factory : IFetiDPSubdomainMatrixManagerFactory<CsrMatrix>
		{
			public ISubdomainMatrixAssembler<CsrMatrix> CreateAssembler() => new CsrMatrixAssembler(false);

			public IFetiDPSubdomainMatrixManager CreateMatrixManager(
				SubdomainLinearSystem<CsrMatrix> linearSystem, FetiDPSubdomainDofs subdomainDofs)
				=> new FetiDPSubdomainMatrixManagerCSparse(linearSystem, subdomainDofs);
		}
	}
}
