using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;

namespace MGroup.Environments.Mpi
{
	/// <summary>
	/// All MPI processes store the same global data and execute the same global operations. If there are extra processes that
	/// do not accommodate compute nodes, then they will be ignored.
	/// </summary>
	public class DemocraticGlobalOperationStrategy : IMpiGlobalOperationStrategy
	{
		public DemocraticGlobalOperationStrategy()
		{
		}

		public void DoGlobalOperation(MpiEnvironment environment, Action globalOperation)
		{
			// Extra processes, if any, do not take part in global operations.
			if (environment.CommNodes == null)
			{
				return;
			}

			// The global operation is run on all processes.
			globalOperation();
		}

		public Dictionary<int, T> ExtractNodeDataFromGlobalToLocalMemories<T>(
			MpiEnvironment environment, Func<int, T> subdomainOperation)
		{
			// Extra processes, if any, do not take part in global operations.
			if (environment.CommNodes == null)
			{
				return null;
			}

			// On each process, we will only deal with the local nodes.
			int clusterID = environment.CommNodes.Rank;
			Dictionary<int, ComputeNode> nodes = environment.NodeTopology.Clusters[clusterID].Nodes;

			// Add the keys first to avoid race conditions.
			var nodeData = new Dictionary<int, T>(nodes.Count);
			foreach (int nodeID in nodes.Keys)
			{
				nodeData[nodeID] = default;
			}

			// Calculate data of each local node in parallel.
			Parallel.ForEach(nodes.Keys, nodeID => nodeData[nodeID] = subdomainOperation(nodeID));

			// No need to scatter anything, since everything done so far was on the memory of the appropriate process.
			return nodeData;
		}

		public Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(MpiEnvironment environment, Func<int, T> getLocalNodeData)
		{
			// Extra processes, if any, do not take part in global operations.
			if (environment.CommNodes == null)
			{
				return null;
			}

			// Each process gathers data corresponding to all compute nodes.
			Dictionary<int, T> globalNodeDataStorage = environment.AllGather(getLocalNodeData);
			return globalNodeDataStorage;
		}

		public Dictionary<int, T> TransferNodeDataToLocalMemories<T>(
			MpiEnvironment environment, Dictionary<int, T> globalNodeDataStorage)
		{
			// Extra processes, if any, do not take part in global operations.
			if (environment.CommNodes == null)
			{
				return null;
			}

			// No need to scatter anything, since global memory is the same as process memory. On the other hand,
			// process memory contains data for all nodes. Return only the data that correspond to local nodes of the process. 
			int clusterID = environment.CommNodes.Rank;
			ComputeNodeCluster cluster = environment.NodeTopology.Clusters[clusterID];
			var localNodeData = new Dictionary<int, T>(cluster.Nodes.Count);
			foreach (int n in cluster.Nodes.Keys)
			{
				localNodeData[n] = globalNodeDataStorage[n];
			}

			return localNodeData;
		}
	}
}
