using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Environments.Mpi
{
	/// <summary>
	/// All MPI processes store the same global data and execute the same global operations.
	/// </summary>
	public class DemocraticGlobalOperationStrategy : IMpiGlobalOperationStrategy
	{
		public DemocraticGlobalOperationStrategy()
		{
		}

		public MpiEnvironment Environment { get; set; }

		public void DoGlobalOperation(Action globalOperation)
		{
			if (Environment.CommNodes == null)
			{
				return;
			}

			//Console.WriteLine($"Process {Environment.CommWorld.Rank}: Executing the global operation");
			//Environment.CommWorld.Barrier();
			globalOperation();
		}

		public Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(Func<int, T> getLocalNodeData)
		{
			Dictionary<int, T> globalNodeDataStorage = Environment.AllGather(getLocalNodeData, Environment.CommNodes);
			//Console.WriteLine($"Process {Environment.CommWorld.Rank}: Global data from gather is null = {globalNodeDataStorage == null}");
			//Environment.CommWorld.Barrier();
			return globalNodeDataStorage;
		}

		public Dictionary<int, T> TransferNodeDataToLocalMemories<T>(Dictionary<int, T> globalNodeDataStorage)
		{
			if (Environment.CommNodes == null)
			{
				return null;
			}

			//Console.WriteLine($"Process {Environment.CommWorld.Rank}: Global data to scatter is null = {globalNodeDataStorage == null}");
			//Environment.CommNodes.Barrier();

			// No need to scatter anything, since global memory is the same as process memory
			int clusterID = Environment.CommNodes.Rank;
			ComputeNodeCluster cluster = Environment.NodeTopology.Clusters[clusterID];
			var localNodeData = new Dictionary<int, T>(cluster.Nodes.Count);
			foreach (int n in cluster.Nodes.Keys)
			{
				localNodeData[n] = globalNodeDataStorage[n];
			}
			//Console.WriteLine($"Process {Environment.CommWorld.Rank}: Count of local data from scatter = {localNodeData.Count}");
			//Environment.CommNodes.Barrier();
			return localNodeData;
		}
	}
}
