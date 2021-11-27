using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices.Operators;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.LinearSystem;

namespace MGroup.Solvers.DDM.FetiDP.Vectors
{
	public class FetiDPSubdomainVectors
	{
		private readonly ISubdomainLinearSystem linearSystem;
		private readonly IFetiDPSubdomainMatrixManager matrixManagerFetiDP;
		private readonly FetiDPSubdomainDofs subdomainDofs;
		private readonly SubdomainLagranges subdomainLagranges;

		public Vector VectorFc { get; private set; }

		public Vector VectorFcCondensed { get; private set; }

		public Vector VectorFr { get; private set; }

		public Vector VectorInvKrrTimesFr { get; private set; }

		public FetiDPSubdomainVectors(ISubdomainLinearSystem linearSystem, FetiDPSubdomainDofs subdomainDofs, 
			SubdomainLagranges subdomainLagranges, IFetiDPSubdomainMatrixManager matrixManagerFetiDP)
		{
			this.linearSystem = linearSystem;
			this.subdomainDofs = subdomainDofs;
			this.subdomainLagranges = subdomainLagranges;
			this.matrixManagerFetiDP = matrixManagerFetiDP;
		}

		public bool IsEmpty => VectorFr == null;

		public void CalcCondensedRhsVector()
		{
			// Static condensation: fcCondensed[s] = fc[s] - Kcr[s] * inv(Krr[s]) * fr[s]
			VectorInvKrrTimesFr = matrixManagerFetiDP.MultiplyInverseKrrTimes(VectorFr);
			VectorFcCondensed = VectorFc - matrixManagerFetiDP.MultiplyKcrTimes(VectorInvKrrTimesFr);
		}

		public void Clear()
		{
			VectorFc = null;
			VectorFcCondensed = null;
			VectorFr = null;
			VectorInvKrrTimesFr = null;
		}

		public void ExtractBoundaryInternalRhsVectors(Action<Vector> scaleBoundaryVector)
		{
			int[] remainderDofs = subdomainDofs.DofsRemainderToFree;
			int[] cornerDofs = subdomainDofs.DofsCornerToFree;

			Vector ff = linearSystem.RhsVector.Copy();
			scaleBoundaryVector(ff);

			this.VectorFr = ff.GetSubvector(remainderDofs);
			this.VectorFc = ff.GetSubvector(cornerDofs);
		}

		public Vector CalcSubdomainFreeSolution(Vector subdomainCornerDisplacements, Vector subdomainLagrangeValues)
		{
			int numFreeDofs = linearSystem.DofOrdering.NumFreeDofs;
			int[] remainderDofs = subdomainDofs.DofsRemainderToFree;
			int[] cornerDofs = subdomainDofs.DofsCornerToFree;

			// ur[s] = inv(Krr[s]) * (fr[s] - Dr[s]^T * lambda[s] - Krc[s] * uc[s])
			Vector uc = subdomainCornerDisplacements;
			Vector lambda = subdomainLagrangeValues;
			SignedBooleanMatrixRowMajor Dr = subdomainLagranges.MatrixDr;
			Vector v1 = VectorFc.Copy();
			Vector v2 = Dr.Multiply(lambda, true);
			Vector v3 = matrixManagerFetiDP.MultiplyKrcTimes(uc);
			v1.SubtractIntoThis(v2);
			v1.SubtractIntoThis(v3);
			Vector ur = matrixManagerFetiDP.MultiplyInverseKrrTimes(v1);

			// Gather ub[s], ui[s] into uf[s]
			var uf = Vector.CreateZero(numFreeDofs);
			uf.CopyNonContiguouslyFrom(remainderDofs, ur);
			uf.CopyNonContiguouslyFrom(cornerDofs, uc);

			return uf;
		}

	}
}
