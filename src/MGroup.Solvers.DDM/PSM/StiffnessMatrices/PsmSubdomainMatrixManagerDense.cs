using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.PSM.Dofs;
using MGroup.Solvers.DDM.StiffnessMatrices;

namespace MGroup.Solvers.DDM.PSM.StiffnessMatrices
{
	public class PsmSubdomainMatrixManagerDense : IPsmSubdomainMatrixManager
	{
		private readonly PsmSubdomainDofs subdomainDofs;
		private readonly SubdomainMatrixManagerDense managerBasic;

		private Matrix Kbb;
		private Matrix Kbi;
		private Matrix Kib;
		private Matrix Kii;
		private Matrix inverseKii;

		public PsmSubdomainMatrixManagerDense(PsmSubdomainDofs subdomainDofs, SubdomainMatrixManagerDense managerBasic)
		{
			this.subdomainDofs = subdomainDofs;
			this.managerBasic = managerBasic;
		}

		public IMatrixView CalcSchurComplement() => Kbb - Kbi * (inverseKii * Kib);

		public void ClearSubMatrices()
		{
			Kbb = null;
			Kbi = null;
			Kib = null;
			Kii = null;
			inverseKii = null;
		}

		public void ExtractKiiKbbKib()
		{
			Matrix Kff = managerBasic.MatrixKff;
			int[] boundaryDofs = subdomainDofs.DofsBoundaryToFree;
			int[] internalDofs = subdomainDofs.DofsInternalToFree;
			Kbb = Kff.GetSubmatrix(boundaryDofs, boundaryDofs);
			Kbi = Kff.GetSubmatrix(boundaryDofs, internalDofs);
			Kib = Kff.GetSubmatrix(internalDofs, boundaryDofs);
			Kii = Kff.GetSubmatrix(internalDofs, internalDofs);
		}

		public void HandleDofsWereModified() => ClearSubMatrices();

		public void InvertKii()
		{
			inverseKii = Kii.Invert();
			Kii = null; // Kii has been overwritten
		}

		public Vector MultiplyInverseKii(Vector vector) => inverseKii * vector;

		public Vector MultiplyKbb(Vector vector) => Kbb * vector;

		public Vector MultiplyKbi(Vector vector) => Kbi * vector;

		public Vector MultiplyKib(Vector vector) => Kib * vector;

		public void ReorderInternalDofs() => subdomainDofs.ReorderInternalDofs(DofPermutation.CreateNoPermutation());

		public class Factory : IPsmSubdomainMatrixManagerFactory
		{
			public (ISubdomainMatrixManager, IPsmSubdomainMatrixManager) CreateMatrixManagers(
				ISubdomain subdomain, PsmSubdomainDofs subdomainDofs)
			{
				var basicMatrixManager = new SubdomainMatrixManagerDense(subdomain);
				var psmMatrixManager = new PsmSubdomainMatrixManagerDense(subdomainDofs, basicMatrixManager);
				return (basicMatrixManager, psmMatrixManager);
			}
		}
	}
}
