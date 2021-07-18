using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
	public class PsmSubdomainVectors
	{
		private readonly IModel model;
		private readonly PsmSubdomainDofs dofs;
		private readonly PsmSubdomainStiffnesses stiffnesses;

		public PsmSubdomainVectors(IModel model, PsmSubdomainDofs dofs, PsmSubdomainStiffnesses stiffnesses)
		{
			this.model = model;
			this.dofs = dofs;
			this.stiffnesses = stiffnesses;
		}

		public BlockVector Fbe { get; set; }

		public Dictionary<int, Vector> Fb { get; } = new Dictionary<int, Vector>();

		public BlockVector Fie { get; set; }

		public Dictionary<int, Vector> Fi { get; } = new Dictionary<int, Vector>();

		public BlockVector FbeCondensed { get; set; }

		public Dictionary<int, Vector> FbCondensed { get; } = new Dictionary<int, Vector>();

		public Dictionary<int, Vector> Uf { get; } = new Dictionary<int, Vector>();

		public void CalcAllRhsVectors(Func<int, Vector> getSubdomainFf, Action<int, Vector> scaleFb)
		{
			Fbe = new BlockVector(dofs.NumSubdomainDofsBoundary);
			Fie = new BlockVector(dofs.NumSubdomainDofsInternal);
			FbeCondensed = new BlockVector(dofs.NumSubdomainDofsBoundary);
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				int[] boundaryToFree = dofs.SubdomainDofsBoundaryToFree[s];
				int[] internalToFree = dofs.SubdomainDofsInternalToFree[s];
				Vector Ff = getSubdomainFf(s);

				Fb[s] = Ff.GetSubvector(boundaryToFree);
				scaleFb(s, Fb[s]);
				Fi[s] = Ff.GetSubvector(internalToFree);
				FbCondensed[s] = Fb[s] - stiffnesses.Kbi[s] * stiffnesses.InvKii[s] * Fi[s];

				Fbe.AddBlock(s, Fb[s]);
				Fie.AddBlock(s, Fi[s]);
				FbeCondensed.AddBlock(s, FbCondensed[s]);
			}
		}

		public void CalcFreeDisplacements(int s, Vector Ub)
		{
			// Extract internal and boundary parts of rhs vector 
			int numFreeDofs = dofs.NumSubdomainDofsFree[s];
			int[] boundaryToFree = dofs.SubdomainDofsBoundaryToFree[s];
			int[] internalToFree = dofs.SubdomainDofsInternalToFree[s];

			// ui[s] = inv(Kii[s]) * (fi[s] - Kib[s] * ub[s])
			Vector Ui = stiffnesses.InvKii[s] * (Fi[s] - stiffnesses.Kib[s] * Ub); 

			// Gather ub[s], ui[s] into uf[s]
			Uf[s] = Vector.CreateZero(numFreeDofs);
			Uf[s].CopyNonContiguouslyFrom(boundaryToFree, Ub);
			Uf[s].CopyNonContiguouslyFrom(internalToFree, Ui);
		}
	}
}
