using System;
using System.Collections.Generic;

using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
	public class HomogeneousScaling : IPsmScaling
	{
		private readonly IModel model;
		private readonly DistributedAlgebraicModel<Matrix> algebraicModel;
		private readonly PsmSubdomainDofs dofs;

		public Dictionary<int, double[]> SudomainInverseMultiplicities { get; } = new Dictionary<int, double[]>();

		public HomogeneousScaling(IModel model, DistributedAlgebraicModel<Matrix> algebraicModel, PsmSubdomainDofs dofs)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.dofs = dofs;
		}

		public void CalcScalingMatrices(Func<int, Matrix> getKff)
		{
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				DofTable boundaryDofs = dofs.SubdomainDofOrderingBoundary[s];
				var inverseMultiplicities = new double[dofs.NumSubdomainDofsBoundary[s]];
				foreach ((INode node, _, int idx) in boundaryDofs)
				{
					inverseMultiplicities[idx] = 1.0 / node.SubdomainsDictionary.Count;
				}
				this.SudomainInverseMultiplicities[s] = inverseMultiplicities;
			}
		}

		public void ScaleRhsVector(int subdomainID, Vector Fb)
		{
			double[] coeffs = this.SudomainInverseMultiplicities[subdomainID];
			for (int i = 0; i < coeffs.Length; ++i)
			{
				Fb[i] *= coeffs[i];
			}
		}
	}
}
