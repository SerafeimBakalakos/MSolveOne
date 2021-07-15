using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Triangulation;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.LinearAlgebraExtensions;
using MGroup.Solvers.DDM.PSM.Dofs;
using MGroup.Solvers.DDM.StiffnessMatrices;

namespace MGroup.Solvers.DDM.PSM.StiffnessMatrices
{
	public class PsmSubdomainMatrixManagerCSparse : IPsmSubdomainMatrixManager
	{
		private readonly PsmSubdomainDofs subdomainDofs;
		private readonly SubdomainMatrixManagerCsr managerBasic;
		private readonly SubmatrixExtractorCsrCsc submatrixExtractor = new SubmatrixExtractorCsrCsc();

		private CsrMatrix Kbb;
		private CsrMatrix Kbi;
		private CsrMatrix Kib;
		private CscMatrix Kii;
		private LUCSparseNet inverseKii;

		public PsmSubdomainMatrixManagerCSparse(PsmSubdomainDofs subdomainDofs, SubdomainMatrixManagerCsr managerBasic)
		{
			this.subdomainDofs = subdomainDofs;
			this.managerBasic = managerBasic;
		}

		public IMatrixView CalcSchurComplement()
		{
			//TODO: Implement a ScurComplement class where A11 is in CSR format
			Matrix kbb = Kbb.CopyToFullMatrix();
			return SchurComplementFullCsrCsc.CalcSchurComplement(kbb, Kbi, Kib, inverseKii);
		}

		public void ClearSubMatrices()
		{
			Kbb = null;
			Kbi = null;
			Kib = null;
			Kii = null;
			inverseKii = null;
		}

		//TODO: Optimize this method. It is too slow.
		public void ExtractKiiKbbKib()
		{
			int[] boundaryDofs = subdomainDofs.DofsBoundaryToFree;
			int[] internalDofs = subdomainDofs.DofsInternalToFree;

			CsrMatrix Kff = managerBasic.MatrixKff;
			submatrixExtractor.ExtractSubmatrices(Kff, boundaryDofs, internalDofs);
			Kbb = submatrixExtractor.Submatrix00;
			Kbi = submatrixExtractor.Submatrix01;
			Kib = submatrixExtractor.Submatrix10;
			Kii = submatrixExtractor.Submatrix11;
		}

		public void HandleDofsWereModified()
		{
			ClearSubMatrices();
			submatrixExtractor.Clear();
		}

		public void InvertKii()
		{
			inverseKii = LUCSparseNet.Factorize(Kii);
			Kii = null; // This memory is not overwritten, but it is not needed anymore either.
		}

		public Vector MultiplyInverseKii(Vector vector) => inverseKii.SolveLinearSystem(vector);

		public Vector MultiplyKbb(Vector vector) => Kbb * vector;

		public Vector MultiplyKbi(Vector vector) => Kbi.Multiply(vector, false);

		public Vector MultiplyKib(Vector vector) => Kib.Multiply(vector, false);

		public void ReorderInternalDofs() => subdomainDofs.ReorderInternalDofs(DofPermutation.CreateNoPermutation());

		public class Factory : IPsmSubdomainMatrixManagerFactory
		{
			public (ISubdomainMatrixManager, IPsmSubdomainMatrixManager) CreateMatrixManagers(
				ISubdomain subdomain, PsmSubdomainDofs subdomainDofs)
			{
				var basicMatrixManager = new SubdomainMatrixManagerCsr(subdomain, false);
				var psmMatrixManager = new PsmSubdomainMatrixManagerCSparse(subdomainDofs, basicMatrixManager);
				return (basicMatrixManager, psmMatrixManager);
			}
		}
	}
}
