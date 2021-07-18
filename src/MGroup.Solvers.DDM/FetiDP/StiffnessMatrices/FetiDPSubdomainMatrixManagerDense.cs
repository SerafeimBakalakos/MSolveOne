using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.Assemblers;

namespace MGroup.Solvers.DDM.FetiDP.StiffnessMatrices
{
	public class FetiDPSubdomainMatrixManagerDense : IFetiDPSubdomainMatrixManager
	{
		private readonly SubdomainLinearSystem<Matrix> linearSystem;
		private readonly FetiDPSubdomainDofs subdomainDofs;

		private Matrix Kcc;
		private Matrix Kcr;
		private Matrix Krc;
		private Matrix Krr;
		private Matrix inverseKrr;
		private Matrix Scc;

		public FetiDPSubdomainMatrixManagerDense(SubdomainLinearSystem<Matrix> linearSystem, FetiDPSubdomainDofs subdomainDofs)
		{
			this.linearSystem = linearSystem;
			this.subdomainDofs = subdomainDofs;
		}

		public IMatrix SchurComplementOfRemainderDofs => Scc;

		public void CalcSchurComplementOfRemainderDofs()
		{
			Scc = Kcc - (Kcr * (inverseKrr * Krc));
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
			Matrix Kff = linearSystem.Matrix;
			Kcc = Kff.GetSubmatrix(cornerToFree, cornerToFree);
			Kcr = Kff.GetSubmatrix(cornerToFree, remainderToFree);
			Krc = Kff.GetSubmatrix(remainderToFree, cornerToFree);
			Krr = Kff.GetSubmatrix(remainderToFree, remainderToFree);
		}

		public void HandleDofsWereModified() => ClearSubMatrices();

		public void InvertKrr()
		{
			inverseKrr = Krr.Invert();
			Krr = null; // Krr has been overwritten
		}

		public Vector MultiplyInverseKrrTimes(Vector vector) => inverseKrr * vector;

		public Vector MultiplyKccTimes(Vector vector) => Kcc * vector;

		public Vector MultiplyKcrTimes(Vector vector) => Kcr * vector;

		public Vector MultiplyKrcTimes(Vector vector) => Krc * vector;

		public void ReorderRemainderDofs() => subdomainDofs.ReorderRemainderDofs(DofPermutation.CreateNoPermutation());

		public class Factory : IFetiDPSubdomainMatrixManagerFactory<Matrix>
		{
			public ISubdomainMatrixAssembler<Matrix> CreateAssembler() => new DenseMatrixAssembler();

			public IFetiDPSubdomainMatrixManager CreateMatrixManager(
				SubdomainLinearSystem<Matrix> linearSystem, FetiDPSubdomainDofs subdomainDofs)
				=> new FetiDPSubdomainMatrixManagerDense(linearSystem, subdomainDofs);
		}
	}
}
