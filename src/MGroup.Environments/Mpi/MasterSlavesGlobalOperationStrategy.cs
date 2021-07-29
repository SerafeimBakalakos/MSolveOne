using System;
using System.Collections.Generic;
using System.Text;

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
			if (environment.CommWorld.Rank == masterProcessRank)
			{
				//Console.WriteLine($"Process {environment.CommWorld.Rank}: Executing the global operation");
				globalOperation();
			}
			//environment.CommWorld.Barrier();
		}

		public Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(MpiEnvironment environment, Func<int, T> getLocalNodeData)
		{
			Dictionary<int, T> globalNodeDataStorage = environment.GatherToRootProcess(getLocalNodeData, masterProcessRank);
			//Console.WriteLine($"Process {environment.CommWorld.Rank}: Global data from gather is null = {globalNodeDataStorage == null}");
			//environment.CommWorld.Barrier();
			return globalNodeDataStorage;
		}

		public Dictionary<int, T> TransferNodeDataToLocalMemories<T>(
			MpiEnvironment environment, Dictionary<int, T> globalNodeDataStorage)
		{
			//Console.WriteLine($"Process {environment.CommWorld.Rank}: Global data to scatter is null = {globalNodeDataStorage == null}");
			//environment.CommWorld.Barrier();
			Dictionary<int, T> localNodeData = environment.ScatterFromRootProcess(globalNodeDataStorage, masterProcessRank);
			//Console.WriteLine($"Process {environment.CommWorld.Rank}: Count of local data from scatter = {localNodeData.Count}");
			return localNodeData;
		}
	}
}
