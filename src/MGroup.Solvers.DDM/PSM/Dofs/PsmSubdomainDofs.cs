using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.Commons;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DofOrdering;

//TODOMPI: DofTable should be replaced with an IntTable that stores ids, instead of actual references to nodes and dofs. 
//		This will make transfering it via MPI much faster.
//TODO: Naming convention for dofs (free/constrained, boundary/internal/corner/intercluster, subdomain/cluster/global) that will
//		be followed across all components
//TODOMPI: Replace DofTable with an equivalent class that uses integers. Also allow clients to choose sorted versions
namespace MGroup.Solvers.DDM.PSM.Dofs
{
	public class PsmSubdomainDofs
	{
		private readonly ActiveDofs allDofs;
		private readonly ISubdomainLinearSystem linearSystem;

		//TODO: This is essential for testing and very useful for debugging, but not production code. Should I remove it?
		private readonly bool sortDofsWhenPossible;

		public PsmSubdomainDofs(ActiveDofs allDofs, ISubdomainLinearSystem linearSystem, bool sortDofsWhenPossible = false)
		{
			this.allDofs = allDofs;
			this.linearSystem = linearSystem;
			this.sortDofsWhenPossible = sortDofsWhenPossible;
		}

		public IntDofTable DofOrderingBoundary { get; private set; }

		public int[] DofsBoundaryToFree { get; private set; }

		public int[] DofsInternalToFree { get; private set; }

		public int NumFreeDofs { get; private set; }
		
		public void ReorderInternalDofs(DofPermutation permutation)
		{
			if (permutation.IsBetter)
			{
				DofsInternalToFree = permutation.ReorderKeysOfDofIndicesMap(DofsInternalToFree);
			}
		}

		/// <summary>
		/// Boundary/internal dofs
		/// </summary>
		public void SeparateFreeDofsIntoBoundaryAndInternal()
		{
			//TODOMPI: force sorting per node and dof
			var boundaryDofOrdering = new IntDofTable();
			var boundaryToFree = new List<int>();
			var internalToFree = new HashSet<int>();
			int subdomainBoundaryIdx = 0;

			DofTable freeDofs = linearSystem.DofOrdering.FreeDofs;
			IEnumerable<INode> nodes = freeDofs.GetRows();
			if (sortDofsWhenPossible)
			{
				nodes = nodes.OrderBy(node => node.ID);
			}

			foreach (INode node in nodes) //TODO: Optimize access: Directly get INode, Dictionary<IDof, int>
			{
				IReadOnlyDictionary<IDofType, int> dofsOfNode = freeDofs.GetDataOfRow(node);
				if (sortDofsWhenPossible)
				{
					var sortedDofsOfNode = new SortedDictionary<IDofType, int>(new DofTypeComparer(allDofs));
					foreach (var dofTypeIdxPair in dofsOfNode)
					{
						sortedDofsOfNode[dofTypeIdxPair.Key] = dofTypeIdxPair.Value;
					}
					dofsOfNode = sortedDofsOfNode;
				}

				if (node.Subdomains.Count > 1)
				{
					foreach (var dofTypeIdxPair in dofsOfNode)
					{
						int dofID = allDofs.GetIdOfDof(dofTypeIdxPair.Key);
						boundaryDofOrdering[node.ID, dofID] = subdomainBoundaryIdx++;
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

			this.DofOrderingBoundary = boundaryDofOrdering;
			this.DofsBoundaryToFree = boundaryToFree.ToArray();
			this.DofsInternalToFree = internalToFree.ToArray();
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
