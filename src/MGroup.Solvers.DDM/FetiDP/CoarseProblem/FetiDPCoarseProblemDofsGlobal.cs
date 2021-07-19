using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.Mappings;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemDofsGlobal
	{
		private readonly IComputeEnvironment environment;
		private readonly IModel model;
		private readonly Func<int, FetiDPSubdomainDofs> getSubdomainDofs;

		public FetiDPCoarseProblemDofsGlobal(IComputeEnvironment environment, IModel model, 
			Func<int, FetiDPSubdomainDofs> getSubdomainDofs)
		{
			this.environment = environment;
			this.model = model;
			this.getSubdomainDofs = getSubdomainDofs;
		}

		public DofTable GlobalDofOrderingCorner { get; private set; }

		public int NumGlobalCornerDofs { get; private set; }

		//TODOMPI: This class should not be coupled with the knowledge that these are data transfered from remote processes and 
		//		thus should be cached 
		public ConcurrentDictionary<int, DofTable> SubdomainDofOrderingsCorner { get; }
			= new ConcurrentDictionary<int, DofTable>();

		public ConcurrentDictionary<int, BooleanMatrixRowsToColumns> SubdomainMatricesLc { get; } 
			= new ConcurrentDictionary<int, BooleanMatrixRowsToColumns>();

		/// <summary>
		/// Essentially the same data as in <see cref="SubdomainMatricesLc"/>.
		/// </summary>
		public ConcurrentDictionary<int, int[]> SubdomainToGlobalCornerDofMaps { get; }
			= new ConcurrentDictionary<int, int[]>();

		public void FindGlobalCornerDofs()
		{
			// Transfer subdomain corner dofs
			environment.DoPerNode(subdomainID =>
			{
				SubdomainDofOrderingsCorner[subdomainID] = getSubdomainDofs(subdomainID).DofOrderingCorner;
			});

			// Aggregate them
			GlobalDofOrderingCorner = new DofTable();
			int numCornerDofs = 0;
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				foreach ((INode node, IDofType dof, int idx) in SubdomainDofOrderingsCorner[subdomain.ID])
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
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				int numSubdomainDofs = SubdomainDofOrderingsCorner[s].EntryCount;
				var subdomainToGlobalMap = new int[numSubdomainDofs];
				foreach ((INode node, IDofType dof, int subdomainIdx) in SubdomainDofOrderingsCorner[s])
				{
					int globalIdx = GlobalDofOrderingCorner[node, dof];
					subdomainToGlobalMap[subdomainIdx] = globalIdx;
				}
				var Lc = new BooleanMatrixRowsToColumns(numSubdomainDofs, NumGlobalCornerDofs, subdomainToGlobalMap);
				SubdomainMatricesLc[s] = Lc;
				SubdomainToGlobalCornerDofMaps[s] = subdomainToGlobalMap;
			}
		}
	}
}
