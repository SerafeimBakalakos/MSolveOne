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
		private readonly SubdomainLinearSystem<SymmetricCscMatrix> linearSystem;
		private readonly FetiDPSubdomainDofs subdomainDofs;
		private readonly OrderingAmdSuiteSparse reordering = new OrderingAmdSuiteSparse();
		private readonly SubmatrixExtractorPckCsrCscSym submatrixExtractor = new SubmatrixExtractorPckCsrCscSym();

		private SymmetricMatrix Kcc;
		private CsrMatrix Kcr;
		private SymmetricCscMatrix Krr;
		private CholeskySuiteSparse inverseKrr;
		private SymmetricMatrix Scc;

		public FetiDPSubdomainMatrixManagerSymmetricSuiteSparse(
			SubdomainLinearSystem<SymmetricCscMatrix> linearSystem, FetiDPSubdomainDofs subdomainDofs)
		{
			this.linearSystem = linearSystem;
			this.subdomainDofs = subdomainDofs;
		}

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
		}

		public void HandleDofsWereModified()
		{
			ClearSubMatrices();
			submatrixExtractor.Clear();
		}

		public void InvertKrr()
		{
			if (inverseKrr != null)
			{
				inverseKrr.Dispose();
			}
			inverseKrr = CholeskySuiteSparse.Factorize(Krr, true);
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
