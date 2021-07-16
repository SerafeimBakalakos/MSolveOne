using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.Commons;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PSM.Dofs;

//TODOMPI: Instead of asking ComputeNode for its neighbors, ISubdomain should contain this info. 
//TODOMPI: I should decouple the code, such that there are classes that define operations on data, classes that define transfer of data,
//		and classes that define the order of such operations and transfers (sequential, shared parallel or distributed parallel)
//		and any synchronization (e.g. locks in IDofSeparator classes). This should probably be done for each component 
//		(e.g. IDofSeparator, IMatrixManager, etc.). However if I can find common code (boilerplate), I could probably avoid some 
//		duplication.
namespace MGroup.Solvers.DDM.PSM.InterfaceProblem
{
	public class PsmInterfaceProblemDofs
	{
		private readonly IComputeEnvironment environment;
		private readonly IModel model;
		private readonly SubdomainTopology subdomainTopology;
		private readonly Func<int, PsmSubdomainDofs> getPsmSubdomainDofs;
		private readonly Func<int, ISubdomainLinearSystem> getSubdomainLinearSystem;

		//TODOMPI: should this be stored in each PsmSubdomainDofs object instead? Or maybe not stored at all, since it is only used when creating the indexer.
		/// <summary>
		/// First key: current subdomain. Second key: neighbor subdomain. Value: dofs at common nodes between these 2 subdomains
		/// </summary>
		private readonly ConcurrentDictionary<int, Dictionary<int, DofSet>> commonDofsBetweenSubdomains
			= new ConcurrentDictionary<int, Dictionary<int, DofSet>>();

		public PsmInterfaceProblemDofs(IComputeEnvironment environment, IModel model, SubdomainTopology subdomainTopology,
			Func<int, ISubdomainLinearSystem> getSubdomainLinearSystem, Func<int, PsmSubdomainDofs> getPsmSubdomainDofs)
		{
			this.environment = environment;
			this.model = model;
			this.subdomainTopology = subdomainTopology;
			this.getSubdomainLinearSystem = getSubdomainLinearSystem;
			this.getPsmSubdomainDofs = getPsmSubdomainDofs;
		}

		public DistributedOverlappingIndexer CreateDistributedVectorIndexer()
		{
			var indexer = new DistributedOverlappingIndexer(environment);
			Action<int> initializeIndexer = subdomainID =>
			{
				PsmSubdomainDofs subdomainDofs = getPsmSubdomainDofs(subdomainID);
				int numBoundaryDofs = subdomainDofs.DofsBoundaryToFree.Length;
				DofTable boundaryDofs = subdomainDofs.DofOrderingBoundary;

				var allCommonDofIndices = new Dictionary<int, int[]>();
				foreach (int neighborID in subdomainTopology.GetNeighborsOfSubdomain(subdomainID))
				{
					DofSet commonDofs2 = commonDofsBetweenSubdomains[subdomainID][neighborID];
					var commonDofIndices = new int[commonDofs2.Count()];
					int idx = 0;
					foreach ((int nodeID, int dofID) in commonDofs2.EnumerateNodesDofs()) 
					{
						//TODO: It would be faster to iterate each node and then its dofs. Same for DofTable. 
						//		Even better let DofTable take DofSet as argument and return the indices.
						INode node = model.GetNode(nodeID);
						IDofType dof = AllDofs.GetDofWithId(dofID);
						commonDofIndices[idx++] = boundaryDofs[node, dof];
					}
					allCommonDofIndices[neighborID] = commonDofIndices;
				}

				indexer.GetLocalComponent(subdomainID).Initialize(numBoundaryDofs, allCommonDofIndices);
			};
			environment.DoPerNode(initializeIndexer);
			return indexer;
		}

		public void FindCommonDofsBetweenSubdomains()
		{
			// Find all dofs of each subdomain at the common nodes.
			Action<int> findLocalDofsAtCommonNodes = subdomainID =>
			{
				Dictionary<int, DofSet> commonDofs = FindSubdomainDofsAtCommonNodes(subdomainID);
				commonDofsBetweenSubdomains[subdomainID] = commonDofs;
			};
			environment.DoPerNode(findLocalDofsAtCommonNodes);

			// Send these dofs to the corresponding neighbors and receive theirs.
			Func<int, AllToAllNodeData<int>> prepareDofsToSend = subdomainID =>
			{
				var transferData = new AllToAllNodeData<int>();
				transferData.sendValues = new ConcurrentDictionary<int, int[]>();
				foreach (int neighborID in subdomainTopology.GetNeighborsOfSubdomain(subdomainID))
				{
					DofSet commonDofs = commonDofsBetweenSubdomains[subdomainID][neighborID];

					//TODOMPI: Serialization & deserialization should be done by the environment, if necessary.
					transferData.sendValues[neighborID] = commonDofs.Serialize(); 
				}

				// No buffers for receive values yet, since their lengths are unknown. 
				// Let the environment create them, by using extra communication.
				transferData.recvValues = new ConcurrentDictionary<int, int[]>();
				return transferData;
			};
			Dictionary<int, AllToAllNodeData<int>> transferDataPerSubdomain =
				environment.CreateDictionaryPerNode(prepareDofsToSend);
			environment.NeighborhoodAllToAll(transferDataPerSubdomain, false);

			// Find the intersection between the dofs of a subdomain and the ones received by its neighbor.
			Action<int> processReceivedDofs = subdomainID =>
			{
				AllToAllNodeData<int> transferData = transferDataPerSubdomain[subdomainID];
				foreach (int neighborID in subdomainTopology.GetNeighborsOfSubdomain(subdomainID))
				{
					DofSet receivedDofs = DofSet.Deserialize(transferData.recvValues[neighborID]);
					commonDofsBetweenSubdomains[subdomainID][neighborID] =
						commonDofsBetweenSubdomains[subdomainID][neighborID].IntersectionWith(receivedDofs);
				}
			};
			environment.DoPerNode(processReceivedDofs);
		}

		public PsmSubdomainDofs GetSubdomainDofs(int subdomainID) => getPsmSubdomainDofs(subdomainID);

		private Dictionary<int, DofSet> FindSubdomainDofsAtCommonNodes(int subdomainID)
		{
			var commonDofsOfSubdomain = new Dictionary<int, DofSet>();
			DofTable freeDofs = getSubdomainLinearSystem(subdomainID).DofOrdering.FreeDofs;
			foreach (int neighborID in subdomainTopology.GetNeighborsOfSubdomain(subdomainID))
			{
				var dofSet = new DofSet();
				foreach (int nodeID in subdomainTopology.GetCommonNodesOfSubdomains(subdomainID, neighborID))
				{
					INode node = model.GetNode(nodeID);
					dofSet.AddDofs(node, freeDofs.GetColumnsOfRow(node));
				}
				commonDofsOfSubdomain[neighborID] = dofSet;
			}
			return commonDofsOfSubdomain;
		}
	}
}
