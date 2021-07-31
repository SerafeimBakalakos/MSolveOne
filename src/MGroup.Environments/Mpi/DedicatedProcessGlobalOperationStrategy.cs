using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Environments.Mpi
{
	/// <summary>
	/// One of the MPI processes acts a master, this process is not used to accommodate compute nodes and perform operations per 
	/// node. Global data are stored in this process only. Global operations are executed in this in this process only.
	/// </summary>
	public class DedicatedProcessGlobalOperationStrategy : IMpiGlobalOperationStrategy
	{
		private const int InvalidRank = -1;

		private int masterProcessRank;
		private MpiEnvironment environment;

		public DedicatedProcessGlobalOperationStrategy(int masterProcessRank = InvalidRank)
		{
			this.masterProcessRank = masterProcessRank;
		}

		public MpiEnvironment Environment
		{
			get => environment;
			set
			{
				this.environment = value;
				int numProcessesWithNodes = environment.NodeTopology.Clusters.Count;
				if (numProcessesWithNodes == environment.CommWorld.Size)
				{
					throw new ArgumentException(
						"This global operation strategy must be used when there are more processes than compute node clusters");
				}

				if (masterProcessRank == InvalidRank)
				{
					masterProcessRank = numProcessesWithNodes; // choose the first extra process
				}
				else
				{
					if (masterProcessRank < numProcessesWithNodes)
					{
						throw new ArgumentException($"The master process cannot be rank {masterProcessRank}," +
							$" since that will be used to accommodate and operate on compute nodes.");
					}
				}
			}
		}

		public void DoGlobalOperation(Action globalOperation)
		{
			if (environment.CommWorld.Rank == masterProcessRank)
			{
				//Console.WriteLine($"Process {environment.CommWorld.Rank}: Executing the global operation");
				globalOperation();
			}
			//environment.CommWorld.Barrier();
		}

		public Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(Func<int, T> getLocalNodeData)
		{
			Dictionary<int, T> globalNodeDataStorage = environment.GatherToRootProcess(getLocalNodeData, masterProcessRank);
			//Console.WriteLine($"Process {environment.CommWorld.Rank}: Global data from gather is null = {globalNodeDataStorage == null}");
			//environment.CommWorld.Barrier();
			return globalNodeDataStorage;
		}

		public Dictionary<int, T> TransferNodeDataToLocalMemories<T>(Dictionary<int, T> globalNodeDataStorage)
		{
			//Console.WriteLine($"Process {environment.CommWorld.Rank}: Global data to scatter is null = {globalNodeDataStorage == null}");
			//environment.CommWorld.Barrier();
			Dictionary<int, T> localNodeData = environment.ScatterFromRootProcess(globalNodeDataStorage, masterProcessRank);
			//Console.WriteLine($"Process {environment.CommWorld.Rank}: Count of local data from scatter = {localNodeData.Count}");
			return localNodeData;
		}
	}
}
