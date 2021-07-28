using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Matrices;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Solvers.DDM.FetiDP.Dofs;

namespace MGroup.Solvers.DDM.Prototypes.FetiDP
{
	public class FetiDPSubdomainDofs
	{
		protected readonly IModel model;
		private readonly DistributedAlgebraicModel<Matrix> algebraicModel;
		private readonly ICornerDofSelection cornerDofs;

		public FetiDPSubdomainDofs(IModel model, DistributedAlgebraicModel<Matrix> algebraicModel,
			ICornerDofSelection cornerDofs)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.cornerDofs = cornerDofs;
		}

		public Dictionary<int, int> NumSubdomainDofsCorner { get; } = new Dictionary<int, int>();

		public Dictionary<int, int> NumSubdomainDofsRemainder { get; } = new Dictionary<int, int>();

		public Dictionary<int, IntDofTable> SubdomainDofOrderingCorner { get; } = new Dictionary<int, IntDofTable>();

		public Dictionary<int, int[]> SubdomainDofsCornerToFree { get; } = new Dictionary<int, int[]>();

		public Dictionary<int, int[]> SubdomainDofsRemainderToFree { get; } = new Dictionary<int, int[]>();

		public virtual void FindDofs()
		{
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				SeparateFreeDofsIntoCornerAndRemainder(subdomain);
			}
		}

		protected void SeparateFreeDofsIntoCornerAndRemainder(ISubdomain subdomain)
		{
			var cornerDofOrdering = new IntDofTable();
			var cornerToFree = new List<int>();
			var remainderToFree = new HashSet<int>();
			int subdomainCornerIdx = 0;

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

				foreach (var pair in dofsOfNode)
				{
					int dof = pair.Key;
					int freeDofIdx = pair.Value;
					if (cornerDofs.IsCornerDof(node, dof))
					{
						cornerDofOrdering[node, dof] = subdomainCornerIdx++;
						cornerToFree.Add(freeDofIdx);
					}
					else
					{
						remainderToFree.Add(freeDofIdx);
					}
				}
			}

			int s = subdomain.ID;
			SubdomainDofOrderingCorner[s] = cornerDofOrdering;
			NumSubdomainDofsCorner[s] = cornerToFree.Count;
			NumSubdomainDofsRemainder[s] = remainderToFree.Count;
			SubdomainDofsCornerToFree[s] = cornerToFree.ToArray();
			SubdomainDofsRemainderToFree[s] = remainderToFree.ToArray();
		}

		private class DofTypeComparer : IComparer<IDofType>
		{
			private readonly ActiveDofs allDofs;

			public DofTypeComparer(ActiveDofs allDofs)
			{
				this.allDofs = allDofs;
			}

			public int Compare(IDofType x, IDofType y)
			{
				return allDofs.GetIdOfDof(x) - allDofs.GetIdOfDof(y);
			}
		}
	}
}
