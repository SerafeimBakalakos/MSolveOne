using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Reordering;
using MGroup.LinearAlgebra.Triangulation;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.LinearAlgebraExtensions;
using MGroup.Solvers.DDM.PSM.Dofs;
using MGroup.Solvers.DDM.StiffnessMatrices;

namespace MGroup.Solvers.DDM.PSM.StiffnessMatrices
{
	public class PsmSubdomainMatrixManagerSymmetricSuiteSparse : IPsmSubdomainMatrixManager
	{
		private readonly PsmSubdomainDofs subdomainDofs;
		private readonly SubdomainMatrixManagerSymmetricCsc managerBasic;
		private readonly OrderingAmdSuiteSparse reordering = new OrderingAmdSuiteSparse();
		private readonly SubmatrixExtractorCsrCscSym submatrixExtractor = new SubmatrixExtractorCsrCscSym();

		private CsrMatrix Kbb;
		private CsrMatrix Kbi;
		private SymmetricCscMatrix Kii;
		private CholeskySuiteSparse inverseKii;

		public PsmSubdomainMatrixManagerSymmetricSuiteSparse(PsmSubdomainDofs subdomainDofs,
			SubdomainMatrixManagerSymmetricCsc managerBasic)
		{
			this.subdomainDofs = subdomainDofs;
			this.managerBasic = managerBasic;
		}

		public IMatrixView CalcSchurComplement()
		{
			//TODO: Implement SchurComplement with A11 being in CSR format.
			TriangularUpper kbbUpper = Kbb.CopyToUpperPacked();
			var kbbSymm = SymmetricMatrix.CreateFromPackedColumnMajorArray(kbbUpper.RawData);
			return SchurComplementPckCsrCscSym.CalcSchurComplement(kbbSymm, Kbi, inverseKii);
		}

		public void ClearSubMatrices()
		{
			Kbb = null;
			Kbi = null;
			Kii = null;
			if (inverseKii != null)
			{
				inverseKii.Dispose();
			}
			inverseKii = null;
		}

		//TODO: Optimize this method. It is too slow.
		public void ExtractKiiKbbKib()
		{
			int[] boundaryDofs = subdomainDofs.DofsBoundaryToFree;
			int[] internalDofs = subdomainDofs.DofsInternalToFree;

			SymmetricCscMatrix Kff = managerBasic.MatrixKff;
			submatrixExtractor.ExtractSubmatrices(Kff, boundaryDofs, internalDofs);
			Kbb = submatrixExtractor.Submatrix00;
			Kbi = submatrixExtractor.Submatrix01;
			Kii = submatrixExtractor.Submatrix11;
		}

		public void HandleDofsWereModified()
		{
			ClearSubMatrices();
			submatrixExtractor.Clear();
		}

		public void InvertKii()
		{
			if (inverseKii != null)
			{
				inverseKii.Dispose();
			}
			inverseKii = CholeskySuiteSparse.Factorize(Kii, true);
			Kii = null; // This memory is not overwritten, but it is not needed anymore either.
		}

		public Vector MultiplyInverseKii(Vector vector) => inverseKii.SolveLinearSystem(vector);

		public Vector MultiplyKbb(Vector vector) => Kbb * vector;

		public Vector MultiplyKbi(Vector vector) => Kbi.Multiply(vector, false);

		public Vector MultiplyKib(Vector vector) => Kbi.Multiply(vector, true);

		public void ReorderInternalDofs()
		{
			int[] internalDofs = subdomainDofs.DofsInternalToFree;
			SymmetricCscMatrix Kff = managerBasic.MatrixKff;
			(int[] rowIndicesKii, int[] colOffsetsKii) = submatrixExtractor.ExtractSparsityPattern(Kff, internalDofs);
			bool oldToNew = false; //TODO: This should be provided by the reordering algorithm
			(int[] permutation, _) = reordering.FindPermutation(
				internalDofs.Length, rowIndicesKii.Length, rowIndicesKii, colOffsetsKii);

			subdomainDofs.ReorderInternalDofs(DofPermutation.Create(permutation, oldToNew));
		}

		public class Factory : IPsmSubdomainMatrixManagerFactory
		{
			public (ISubdomainMatrixManager, IPsmSubdomainMatrixManager) CreateMatrixManagers(
				ISubdomain subdomain, PsmSubdomainDofs subdomainDofs)
			{
				var basicMatrixManager = new SubdomainMatrixManagerSymmetricCsc(subdomain);
				var psmMatrixManager = new PsmSubdomainMatrixManagerSymmetricSuiteSparse(subdomainDofs, basicMatrixManager);
				return (basicMatrixManager, psmMatrixManager);
			}
		}
	}
}
