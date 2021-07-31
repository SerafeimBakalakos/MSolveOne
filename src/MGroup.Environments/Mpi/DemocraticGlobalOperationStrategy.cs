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

		public Dictionary<int, T> CalcNodeDataAndTransferToGlobalMemory<T>(MpiEnvironment environment, Func<int, T> calcNodeData)
		{
			// Extra processes, if any, do not take part in global operations.
			if (environment.CommNodes == null)
			{
				return null;
			}

			// Each process gathers data corresponding to all compute nodes.
			Dictionary<int, T> globalNodeDataStorage = environment.AllGather(calcNodeData);
			return globalNodeDataStorage;
		}

		public Dictionary<int, T> CalcNodeDataAndTransferToLocalMemory<T>(MpiEnvironment environment, Func<int, T> calcNodeData)
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
			Parallel.ForEach(nodes.Keys, nodeID => nodeData[nodeID] = calcNodeData(nodeID));

			// No need to scatter anything, since everything done so far was on the memory of the appropriate process.
			return nodeData;
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
	}
}
