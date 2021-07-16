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

namespace MGroup.Solvers.DDM.FetiDP.StiffnessMatrices
{
	public class FetiDPSubdomainMatrixManagerSymmetricCSparse : IFetiDPSubdomainMatrixManager
	{
		private readonly SubdomainLinearSystem<SymmetricCscMatrix> linearSystem;
		private readonly FetiDPSubdomainDofs subdomainDofs;
		private readonly OrderingAmdCSparseNet reordering = new OrderingAmdCSparseNet();
		private readonly SubmatrixExtractorPckCsrCscSym submatrixExtractor = new SubmatrixExtractorPckCsrCscSym();

		private SymmetricMatrix Kcc;
		private SymmetricMatrix KccStar;
		private CsrMatrix Kcr;
		private SymmetricCscMatrix Krr;
		private CholeskyCSparseNet inverseKrr;

		public FetiDPSubdomainMatrixManagerSymmetricCSparse(
			SubdomainLinearSystem<SymmetricCscMatrix> linearSystem, FetiDPSubdomainDofs subdomainDofs)
		{
			this.linearSystem = linearSystem;
			this.subdomainDofs = subdomainDofs;
		}

		public IMatrix SchurComplementOfRemainderDofs => KccStar;

		public void CalcSchurComplementOfRemainderDofs()
		{
			KccStar = SymmetricMatrix.CreateZero(Kcc.Order);
			SchurComplementPckCsrCscSym.CalcSchurComplement(Kcc, Kcr, inverseKrr, KccStar);
		}

		public void ClearSubMatrices()
		{
			inverseKrr = null;
			Kcc = null;
			Kcr = null;
			Krr = null;
			KccStar = null;
		}

		public void ExtractKrrKccKrc()
		{
			int[] cornerToFree = subdomainDofs.DofsCornerToFree;
			int[] remainderToFree = subdomainDofs.DofsRemainderToFree;

			SymmetricCscMatrix Kff = linearSystem.Matrix;
			submatrixExtractor.ExtractSubmatrices(Kff, cornerToFree, remainderToFree);
			Kcc = submatrixExtractor.Submatrix00;
			Kcr = submatrixExtractor.Submatrix01;
			Krr = submatrixExtractor.Submatrix11; 

			//TODO: It would be better if these were returned by the extractor, instead of stored in its properties. 
			//		The only state that the extractor needs is its private mapping arrays
		}

		public void HandleDofsWereModified()
		{
			ClearSubMatrices();
			submatrixExtractor.Clear();
		}

		public void InvertKrr()
		{
			var factorization = CholeskyCSparseNet.Factorize(Krr);
			inverseKrr = factorization;
			Krr = null; // It has not been mutated, but it is no longer needed
		}

		public Vector MultiplyInverseKrrTimes(Vector vector) => inverseKrr.SolveLinearSystem(vector);

		public Vector MultiplyKccTimes(Vector vector) => Kcc * vector;

		public Vector MultiplyKcrTimes(Vector vector) => Kcr * vector;

		public Vector MultiplyKrcTimes(Vector vector) => Kcr.Multiply(vector, true);

		public void ReorderRemainderDofs()
		{
			int[] remainderDofs = subdomainDofs.DofsRemainderToFree;
			SymmetricCscMatrix Kff = linearSystem.Matrix;
			(int[] rowIndicesKrr, int[] colOffsetsKrr) = submatrixExtractor.ExtractSparsityPattern(Kff, remainderDofs);
			(int[] permutation, bool oldToNew) = reordering.FindPermutation(
				remainderDofs.Length, rowIndicesKrr, colOffsetsKrr);

			subdomainDofs.ReorderRemainderDofs(DofPermutation.Create(permutation, oldToNew));
		}

		public class Factory : IFetiDPSubdomainMatrixManagerFactory<SymmetricCscMatrix>
		{
			public ISubdomainMatrixAssembler<SymmetricCscMatrix> CreateAssembler() => new SymmetricCscMatrixAssembler(true);

			public IFetiDPSubdomainMatrixManager CreateMatrixManager(
				SubdomainLinearSystem<SymmetricCscMatrix> linearSystem, FetiDPSubdomainDofs subdomainDofs)
				=> new FetiDPSubdomainMatrixManagerSymmetricCSparse(linearSystem, subdomainDofs);
		}
	}
}
