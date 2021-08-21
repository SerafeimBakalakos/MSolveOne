using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemGlobalDofs
	{
		public FetiDPCoarseProblemGlobalDofs()
		{
		}

		public IntDofTable GlobalDofOrderingCorner { get; private set; }

		public int NumGlobalCornerDofs { get; private set; }

		public Dictionary<int, IntDofTable> SubdomainDofOrderingsCorner { get; private set; }

		public Dictionary<int, int[]> SubdomainToGlobalCornerDofs { get; set; }


		public void FindGlobalCornerDofs(Dictionary<int, IntDofTable> subdomainDofOrderingsCorner)
		{
			// Store them	
			SubdomainDofOrderingsCorner = subdomainDofOrderingsCorner;

			// Aggregate them
			GlobalDofOrderingCorner = new IntDofTable();
			int numCornerDofs = 0;
			foreach (int sub in subdomainDofOrderingsCorner.Keys)
			{
				foreach ((int node, int dof, int idx) in SubdomainDofOrderingsCorner[sub])
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

		public int[] CalcSubdomainGlobalCornerDofMap(int subdomainID)
		{
			int numSubdomainDofs = SubdomainDofOrderingsCorner[subdomainID].EntryCount;
			var subdomainToGlobalMap = new int[numSubdomainDofs];
			foreach ((int node, int dof, int subdomainIdx) in SubdomainDofOrderingsCorner[subdomainID])
			{
				int globalIdx = GlobalDofOrderingCorner[node, dof];
				subdomainToGlobalMap[subdomainIdx] = globalIdx;
			}
			//var Lc = new BooleanMatrixRowsToColumns(numSubdomainDofs, NumGlobalCornerDofs, subdomainToGlobalMap);
			return subdomainToGlobalMap;
		}
	}
}
