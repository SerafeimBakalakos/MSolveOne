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

		public void DoGlobalOperation(MpiEnvironment environment, Action globalOperation)
		{
			//Console.WriteLine($"Process {environment.CommWorld.Rank}: Executing the global operation");
			//environment.CommWorld.Barrier();
			globalOperation();
		}

		public Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(MpiEnvironment environment, Func<int, T> getLocalNodeData)
		{
			Dictionary<int, T> globalNodeDataStorage = environment.AllGather(getLocalNodeData);
			//Console.WriteLine($"Process {environment.CommWorld.Rank}: Global data from gather is null = {globalNodeDataStorage == null}");
			//environment.CommWorld.Barrier();
			return globalNodeDataStorage;
		}

		public Dictionary<int, T> TransferNodeDataToLocalMemories<T>(
			MpiEnvironment environment, Dictionary<int, T> globalNodeDataStorage)
		{
			//Console.WriteLine($"Process {environment.CommWorld.Rank}: Global data to scatter is null = {globalNodeDataStorage == null}");
			//environment.CommWorld.Barrier();

			// No need to scatter anything, since global memory is the same as process memory
			int clusterID = environment.CommWorld.Rank;
			ComputeNodeCluster cluster = environment.NodeTopology.Clusters[clusterID];
			var localNodeData = new Dictionary<int, T>(cluster.Nodes.Count);
			foreach (int n in cluster.Nodes.Keys)
			{
				localNodeData[n] = globalNodeDataStorage[n];
			}
			//Console.WriteLine($"Process {environment.CommWorld.Rank}: Count of local data from scatter = {localNodeData.Count}");
			//environment.CommWorld.Barrier();
			return localNodeData;
		}
	}
}
