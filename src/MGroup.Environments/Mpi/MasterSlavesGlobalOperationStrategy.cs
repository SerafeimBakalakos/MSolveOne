using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Environments.Mpi
{
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
				globalOperation();
			}
		}

		public Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(MpiEnvironment environment, Func<int, T> getLocalNodeData)
			=> environment.GatherToRootProcess(getLocalNodeData, masterProcessRank);

		public Dictionary<int, T> TransferNodeDataToLocalMemories<T>(
			MpiEnvironment environment, Dictionary<int, T> globalNodeDataStorage)
			=> environment.ScatterFromRootProcess(globalNodeDataStorage, masterProcessRank);
	}
}
