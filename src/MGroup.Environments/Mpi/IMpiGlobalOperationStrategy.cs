using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Environments.Mpi
{
	public interface IMpiGlobalOperationStrategy
	{
		void DoGlobalOperation(MpiEnvironment environment, Action globalOperation);

		Dictionary<int, T> ExtractNodeDataFromGlobalToLocalMemories<T>(
			MpiEnvironment environment, Func<int, T> subdomainOperation);

		Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(MpiEnvironment environment, Func<int, T> getLocalNodeData);

		Dictionary<int, T> TransferNodeDataToLocalMemories<T>(
			MpiEnvironment environment, Dictionary<int, T> globalNodeDataStorage);
	}
}
