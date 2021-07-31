using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Threading.Tasks;

namespace MGroup.Environments.Mpi
{
	/// <summary>
	/// One of the MPI processes acts a master. Global data are stored in this process only. Global operations are executed in this 
	/// in this process only.
	/// </summary>
	public class MasterSlavesGlobalOperationStrategy : IMpiGlobalOperationStrategy
	{
		private readonly int masterProcessRank;

		public MasterSlavesGlobalOperationStrategy(int masterProcessRank = 0)
		{
			this.masterProcessRank = masterProcessRank;
		}

		public void DoGlobalOperation(MpiEnvironment environment, Action globalOperation)
		{
			CheckMasterProcessRank(environment);

			// The global operation is run only on master process.
			if (environment.CommWorld.Rank == masterProcessRank)
			{
				globalOperation();
			}
		}

		public Dictionary<int, T> ExtractNodeDataFromGlobalToLocalMemories<T>(
			MpiEnvironment environment, Func<int, T> subdomainOperation)
		{
			CheckMasterProcessRank(environment);

			// Step 1: Calculate data for all nodes, but only in master process, where the callbacks are defined
			Dictionary<int, T> nodeDataInGlobal = null;
			if (environment.CommWorld.Rank == masterProcessRank)
			{
				// On master process we will deal with all nodes.
				Dictionary<int, ComputeNode> nodes = environment.NodeTopology.Nodes;

				// Add the keys first to avoid race conditions.
				nodeDataInGlobal = new Dictionary<int, T>(nodes.Count);
				foreach (int nodeID in nodes.Keys)
				{
					nodeDataInGlobal[nodeID] = default;
				}

				// Calculate data of each node in parallel.
				Parallel.ForEach(nodes.Keys, nodeID => nodeDataInGlobal[nodeID] = subdomainOperation(nodeID));
			}

			// Step 2: Scatter the node data from master process to their corresponding local processes.
			Dictionary<int, T> nodeDataInLocal = environment.ScatterFromRootProcess(nodeDataInGlobal, masterProcessRank);
			return nodeDataInLocal;
		}

		public Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(MpiEnvironment environment, Func<int, T> getLocalNodeData)
		{
			CheckMasterProcessRank(environment);

			// Only the master process gathers data corresponding to all compute nodes.
			Dictionary<int, T> globalNodeDataStorage = environment.GatherToRootProcess(getLocalNodeData, masterProcessRank);
			return globalNodeDataStorage;
		}

		public Dictionary<int, T> TransferNodeDataToLocalMemories<T>(
			MpiEnvironment environment, Dictionary<int, T> globalNodeDataStorage)
		{
			CheckMasterProcessRank(environment);
			Dictionary<int, T> localNodeData = environment.ScatterFromRootProcess(globalNodeDataStorage, masterProcessRank);
			return localNodeData;
		}

		[Conditional("DEBUG")]
		private void CheckMasterProcessRank(MpiEnvironment environment)
		{
			if (masterProcessRank >= environment.CommWorld.Size)
			{
				throw new ArgumentException($"Rank {masterProcessRank} cannot be the master process, " +
					$"when there are {environment.CommWorld.Size} processes in total.");
			}
		}
	}
}
