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

		public MpiEnvironment Environment { get; set; }


		public void DoGlobalOperation(Action globalOperation)
		{
			if (Environment.CommWorld.Rank == masterProcessRank)
			{
				//Console.WriteLine($"Process {Environment.CommWorld.Rank}: Executing the global operation");
				globalOperation();
			}
			//environment.CommWorld.Barrier();
		}

		public Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(Func<int, T> getLocalNodeData)
		{
			Dictionary<int, T> globalNodeDataStorage = Environment.GatherToRootProcess(getLocalNodeData, masterProcessRank);
			//Console.WriteLine($"Process {Environment.CommWorld.Rank}: Global data from gather is null = {globalNodeDataStorage == null}");
			//Environment.CommWorld.Barrier();
			return globalNodeDataStorage;
		}

		public Dictionary<int, T> TransferNodeDataToLocalMemories<T>(Dictionary<int, T> globalNodeDataStorage)
		{
			//Console.WriteLine($"Process {Environment.CommWorld.Rank}: Global data to scatter is null = {globalNodeDataStorage == null}");
			//Environment.CommWorld.Barrier();
			Dictionary<int, T> localNodeData = Environment.ScatterFromRootProcess(globalNodeDataStorage, masterProcessRank);
			//Console.WriteLine($"Process {Environment.CommWorld.Rank}: Count of local data from scatter = {localNodeData.Count}");
			return localNodeData;
		}
	}
}
