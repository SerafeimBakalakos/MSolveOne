using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.Mappings;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemGlobalDofs
	{
		public FetiDPCoarseProblemGlobalDofs()
		{
		}

		public DofTable GlobalDofOrderingCorner { get; private set; }

		public int NumGlobalCornerDofs { get; private set; }

		public Dictionary<int, DofTable> SubdomainDofOrderingsCorner { get; private set; }

		//TODO: This is probably useless, since SubdomainToGlobalCornerDofs is easier to use efficiently
		public Dictionary<int, BooleanMatrixRowsToColumns> SubdomainMatricesLc { get; } 
			= new Dictionary<int, BooleanMatrixRowsToColumns>();

		/// <summary>
		/// Essentially the same data as in <see cref="SubdomainMatricesLc"/>.
		/// </summary>
		public Dictionary<int, int[]> SubdomainToGlobalCornerDofs { get; } = new Dictionary<int, int[]>();

		public void FindGlobalCornerDofs(Dictionary<int, DofTable> subdomainDofOrderingsCorner)
		{
			// Store them	
			SubdomainDofOrderingsCorner = subdomainDofOrderingsCorner;

			// Aggregate them
			GlobalDofOrderingCorner = new DofTable();
			int numCornerDofs = 0;
			foreach (int sub in subdomainDofOrderingsCorner.Keys)
			{
				foreach ((INode node, IDofType dof, int idx) in SubdomainDofOrderingsCorner[sub])
				{
					bool didNotExist = GlobalDofOrderingCorner.TryAdd(node, dof, numCornerDofs);
					if (didNotExist)
					{
						numCornerDofs++;
					}
				}
			}
			NumGlobalCornerDofs = numCornerDofs;
		}

		public void ReorderGlobalCornerDofs(DofPermutation permutation)
		{
			if (permutation.IsBetter)
			{
				// The global dof ordering and the subdomain-global maps need to be updated.
				GlobalDofOrderingCorner.Reorder(permutation.PermutationArray, permutation.PermutationIsOldToNew);
				CalcSubdomainGlobalCornerDofMaps();
			}
		}

		public void CalcSubdomainGlobalCornerDofMaps()
		{
			//TODOMPI: This is parallelizable, but needs a different strategy than the usual one. All these data belong to
			//		a single memory address space, instead of being distributed across separate spaces. In e
			foreach (int sub in SubdomainDofOrderingsCorner.Keys)
			{
				int numSubdomainDofs = SubdomainDofOrderingsCorner[sub].EntryCount;
				var subdomainToGlobalMap = new int[numSubdomainDofs];
				foreach ((INode node, IDofType dof, int subdomainIdx) in SubdomainDofOrderingsCorner[sub])
				{
					int globalIdx = GlobalDofOrderingCorner[node, dof];
					subdomainToGlobalMap[subdomainIdx] = globalIdx;
				}
				var Lc = new BooleanMatrixRowsToColumns(numSubdomainDofs, NumGlobalCornerDofs, subdomainToGlobalMap);
				SubdomainMatricesLc[sub] = Lc;
				SubdomainToGlobalCornerDofs[sub] = subdomainToGlobalMap;
			}
		}
	}
}
