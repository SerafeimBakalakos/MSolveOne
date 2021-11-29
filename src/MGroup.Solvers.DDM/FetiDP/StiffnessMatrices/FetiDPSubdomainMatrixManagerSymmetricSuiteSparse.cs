using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Reordering;
using MGroup.LinearAlgebra.Triangulation;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.LinearAlgebraExtensions;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.Assemblers;

//TODO: a lot of duplication with the CSparse version
namespace MGroup.Solvers.DDM.FetiDP.StiffnessMatrices
{
	public class FetiDPSubdomainMatrixManagerSymmetricSuiteSparse : IFetiDPSubdomainMatrixManager
	{
		/// <summary>
		/// In FETI-DP Krr is also used for the preconditioner. In PFETI-DP only Krr is only used for the Schur complement of 
		/// remainder dofs
		/// </summary>
		private readonly bool clearKrrAfterFactorization = false;
		private readonly SubdomainLinearSystem<SymmetricCscMatrix> linearSystem;
		private readonly FetiDPSubdomainDofs subdomainDofs;
		private readonly OrderingAmdSuiteSparse reordering = new OrderingAmdSuiteSparse();
		private readonly SubmatrixExtractorPckCsrCscSym submatrixExtractorBoundaryInternal = new SubmatrixExtractorPckCsrCscSym();
		private readonly SubmatrixExtractorPckCsrCscSym submatrixExtractorCornerRemainder = new SubmatrixExtractorPckCsrCscSym();

		private SymmetricMatrix Kbb, Kcc;
		private CsrMatrix Kbi, Kcr;
		private SymmetricCscMatrix Kii, Krr;
		private CholeskySuiteSparse inverseKii, inverseKrr;
		private DiagonalMatrix inverseKiiDiagonal;
		private SymmetricMatrix Scc;

		public FetiDPSubdomainMatrixManagerSymmetricSuiteSparse(
			SubdomainLinearSystem<SymmetricCscMatrix> linearSystem, FetiDPSubdomainDofs subdomainDofs)
		{
			this.linearSystem = linearSystem;
			this.subdomainDofs = subdomainDofs;
		}

		public bool IsEmpty => inverseKrr == null;

		public IMatrix SchurComplementOfRemainderDofs => Scc;

		public void CalcSchurComplementOfRemainderDofs()
		{
			Scc = SymmetricMatrix.CreateZero(Kcc.Order);
			SchurComplementPckCsrCscSym.CalcSchurComplement(Kcc, Kcr, inverseKrr, Scc);
		}

		public void ClearSubMatrices()
		{
			if (inverseKrr != null)
			{
				inverseKrr.Dispose();
			}
			inverseKrr = null;
			Kcc = null;
			Kcr = null;
			Krr = null;
			Scc = null;

			if (inverseKii != null)
			{
				inverseKii.Dispose();
			}
			inverseKii = null;
			inverseKiiDiagonal = null;
			Kbb = null;
			Kbi = null;
			Kii = null;
		}

		public void ExtractKiiKbbKib()
		{
			int[] boundaryRemainderToRemainder = subdomainDofs.DofsBoundaryRemainderToRemainder;
			int[] internalToRemainder = subdomainDofs.DofsInternalToRemainder;

			submatrixExtractorBoundaryInternal.ExtractSubmatrices(Krr, boundaryRemainderToRemainder, internalToRemainder);
			Kbb = submatrixExtractorCornerRemainder.Submatrix00;
			Kbi = submatrixExtractorCornerRemainder.Submatrix01;
			Kii = submatrixExtractorCornerRemainder.Submatrix11;
		}

		public void ExtractKrrKccKrc()
		{
			int[] cornerToFree = subdomainDofs.DofsCornerToFree;
			int[] remainderToFree = subdomainDofs.DofsRemainderToFree;

			SymmetricCscMatrix Kff = linearSystem.Matrix;
			submatrixExtractorCornerRemainder.ExtractSubmatrices(Kff, cornerToFree, remainderToFree);
			Kcc = submatrixExtractorCornerRemainder.Submatrix00;
			Kcr = submatrixExtractorCornerRemainder.Submatrix01;
			Krr = submatrixExtractorCornerRemainder.Submatrix11;
		}

		public void HandleDofsWereModified()
		{
			ClearSubMatrices();
			submatrixExtractorCornerRemainder.Clear();
		}

		public void InvertKii(bool diagonalOnly)
		{
			if (diagonalOnly)
			{
				inverseKiiDiagonal = DiagonalMatrix.CreateFromArray(Kii.GetDiagonalAsArray());
				inverseKiiDiagonal.Invert();
			}
			else
			{
				if (inverseKii != null)
				{
					inverseKii.Dispose();
				}
				inverseKii = CholeskySuiteSparse.Factorize(Kii, true);
			}
			Kii = null; // It has not been mutated, but it is no longer needed
		}

		public void InvertKrr()
		{
			if (inverseKrr != null)
			{
				inverseKrr.Dispose();
			}
			inverseKrr = CholeskySuiteSparse.Factorize(Krr, true);
			if (clearKrrAfterFactorization)
			{
				Krr = null; // It has not been mutated, but it is no longer needed
			}
		}

		public Vector MultiplyInverseKiiTimes(Vector vector, bool diagonalOnly)
		{
			if (diagonalOnly)
			{
				return inverseKiiDiagonal.Multiply(vector);
			}
			else
			{
				return inverseKii.SolveLinearSystem(vector);
			}
		}

		public Vector MultiplyInverseKrrTimes(Vector vector) => inverseKrr.SolveLinearSystem(vector);

		public Vector MultiplyKbbTimes(Vector vector) => Kbb * vector;

		public Vector MultiplyKbiTimes(Vector vector) => Kbi * vector;

		public Vector MultiplyKccTimes(Vector vector) => Kcc * vector;

		public Vector MultiplyKcrTimes(Vector vector) => Kcr * vector;

		public Vector MultiplyKibTimes(Vector vector) => Kbi.Multiply(vector, true);

		public Vector MultiplyKrcTimes(Vector vector) => Kcr.Multiply(vector, true);

		public void ReorderInternalDofs()
		{
			throw new NotImplementedException();
		}

		public void ReorderRemainderDofs()
		{
			int[] remainderDofs = subdomainDofs.DofsRemainderToFree;
			SymmetricCscMatrix Kff = linearSystem.Matrix;
			(int[] rowIndicesKrr, int[] colOffsetsKrr) = submatrixExtractorCornerRemainder.ExtractSparsityPattern(Kff, remainderDofs);

			bool oldToNew = false; //TODO: This should be provided by the reordering algorithm
			(int[] permutation, ReorderingStatistics stats) = reordering.FindPermutation(
				remainderDofs.Length, rowIndicesKrr.Length, rowIndicesKrr, colOffsetsKrr);

			subdomainDofs.ReorderRemainderDofs(DofPermutation.Create(permutation, oldToNew));
		}

		public class Factory : IFetiDPSubdomainMatrixManagerFactory<SymmetricCscMatrix>
		{
			public ISubdomainMatrixAssembler<SymmetricCscMatrix> CreateAssembler() => new SymmetricCscMatrixAssembler(true);

			public IFetiDPSubdomainMatrixManager CreateMatrixManager(
				SubdomainLinearSystem<SymmetricCscMatrix> linearSystem, FetiDPSubdomainDofs subdomainDofs)
				=> new FetiDPSubdomainMatrixManagerSymmetricSuiteSparse(linearSystem, subdomainDofs);
		}
	}
}
