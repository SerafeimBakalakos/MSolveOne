using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.LinearSystem;

namespace MGroup.Solvers.DDM.FetiDP.Dofs
{
	public class FetiDPSubdomainDofs
	{
		private readonly ISubdomainLinearSystem linearSystem;

		public FetiDPSubdomainDofs(ISubdomainLinearSystem linearSystem)
		{
			this.linearSystem = linearSystem;
			this.SubdomainID = linearSystem.SubdomainID;
		}

		public IntDofTable DofOrderingCorner { get; private set; }

		public int[] DofsBoundaryRemainderToRemainder { get; private set; }

		public int[] DofsCornerToFree { get; private set; }

		public int[] DofsInternalToRemainder { get; private set; }

		public int[] DofsRemainderToFree { get; private set; }

		public bool IsEmpty => DofOrderingCorner == null;

		public int SubdomainID { get; }

		public void ReorderRemainderDofs(DofPermutation permutation)
		{
			if (permutation.IsBetter)
			{
				DofsRemainderToFree = permutation.ReorderKeysOfDofIndicesMap(DofsRemainderToFree);
			}
		}

		public void SeparateFreeDofsIntoCornerAndRemainder(ICornerDofSelection cornerDofSelection)
		{
			var cornerDofOrdering = new IntDofTable();
			var cornerToFree = new List<int>();
			var remainderToFree = new HashSet<int>();
			int numCornerDofs = 0;
			IntDofTable freeDofs = linearSystem.DofOrdering.FreeDofs;
			IEnumerable<int> nodes = freeDofs.GetRows(); //TODO: Optimize access: Directly get INode, Dictionary<IDof, int>
			foreach (int node in nodes)
			{
				IReadOnlyDictionary<int, int> dofsOfNode = freeDofs.GetDataOfRow(node);
				foreach (var dofIdxPair in dofsOfNode)
				{
					int dof = dofIdxPair.Key;
					if (cornerDofSelection.IsCornerDof(SubdomainID, node, dof))
					{
						cornerDofOrdering[node, dof] = numCornerDofs++;
						cornerToFree.Add(dofIdxPair.Value);
					}
					else
					{
						remainderToFree.Add(dofIdxPair.Value);
					}
				}
			}

			this.DofOrderingCorner = cornerDofOrdering;
			this.DofsCornerToFree = cornerToFree.ToArray();
			this.DofsRemainderToFree = remainderToFree.ToArray();
		}

		public void SeparateRemainderDofsIntoBoundaryAndInternalDofs()
		{
			throw new NotImplementedException("Unused in PFETI-DP. Implement it for FETI-DP");
		}
	}
}
