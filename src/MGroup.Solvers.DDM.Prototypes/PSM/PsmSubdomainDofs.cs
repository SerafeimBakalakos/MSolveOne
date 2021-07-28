using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.LinearSystem;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
	public class PsmSubdomainDofs
	{
		protected readonly IModel model;
		private readonly DistributedAlgebraicModel<Matrix> algebraicModel;

		public PsmSubdomainDofs(IModel model, DistributedAlgebraicModel<Matrix> algebraicModel)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
		}

		public Dictionary<int, int> NumSubdomainDofsBoundary { get; } = new Dictionary<int, int>();

		public Dictionary<int, int> NumSubdomainDofsFree { get; } = new Dictionary<int, int>();

		public Dictionary<int, int> NumSubdomainDofsInternal { get; } = new Dictionary<int, int>();

		public Dictionary<int, IntDofTable> SubdomainDofOrderingBoundary { get; } = new Dictionary<int, IntDofTable>();

		public Dictionary<int, int[]> SubdomainDofsBoundaryToFree { get; } = new Dictionary<int, int[]>();

		public Dictionary<int, int[]> SubdomainDofsInternalToFree { get; } = new Dictionary<int, int[]>();

		public void FindDofs()
		{
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				SeparateFreeDofsIntoBoundaryAndInternal(subdomain);
			}
		}

		protected void SeparateFreeDofsIntoBoundaryAndInternal(ISubdomain subdomain)
		{
			var boundaryDofOrdering = new IntDofTable();
			var boundaryToFree = new List<int>();
			var internalToFree = new HashSet<int>();
			int subdomainBoundaryIdx = 0;

			IntDofTable freeDofs = algebraicModel.SubdomainFreeDofOrderings[subdomain.ID].FreeDofs;
			IEnumerable<int> nodes = freeDofs.GetRows();
			nodes = nodes.OrderBy(node => node);

			foreach (int node in nodes)
			{
				IReadOnlyDictionary<int, int> dofsOfNode = freeDofs.GetDataOfRow(node);
				var sortedDofsOfNode = new SortedDictionary<int, int>();
				foreach (var dofTypeIdxPair in dofsOfNode)
				{
					sortedDofsOfNode[dofTypeIdxPair.Key] = dofTypeIdxPair.Value;
				}
				dofsOfNode = sortedDofsOfNode;

				if (model.GetNode(node).Subdomains.Count > 1)
				{
					foreach (var dofTypeIdxPair in dofsOfNode)
					{
						boundaryDofOrdering[node, dofTypeIdxPair.Key] = subdomainBoundaryIdx++;
						boundaryToFree.Add(dofTypeIdxPair.Value);
					}
				}
				else
				{
					foreach (var dofTypeIdxPair in dofsOfNode)
					{
						internalToFree.Add(dofTypeIdxPair.Value);
					}
				}
			}

			int s = subdomain.ID;
			SubdomainDofOrderingBoundary[s] = boundaryDofOrdering;
			NumSubdomainDofsBoundary[s] = boundaryToFree.Count;
			NumSubdomainDofsInternal[s] = internalToFree.Count;
			NumSubdomainDofsFree[s] = algebraicModel.SubdomainFreeDofOrderings[subdomain.ID].NumFreeDofs;
			SubdomainDofsBoundaryToFree[s] = boundaryToFree.ToArray();
			SubdomainDofsInternalToFree[s] = internalToFree.ToArray();
		}
	}
}
