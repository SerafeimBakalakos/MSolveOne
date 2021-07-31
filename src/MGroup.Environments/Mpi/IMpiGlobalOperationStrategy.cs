using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Environments.Mpi
{
	public interface IMpiGlobalOperationStrategy
	{
		MpiEnvironment Environment { get; set; }

		void DoGlobalOperation(Action globalOperation);

		Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(Func<int, T> getLocalNodeData);

		Dictionary<int, T> TransferNodeDataToLocalMemories<T>(Dictionary<int, T> globalNodeDataStorage);
	}
}
