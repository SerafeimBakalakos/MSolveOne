using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Environments.Mpi
{
	public interface IMpiGlobalOperationStrategy
	{
		Dictionary<int, T> CalcNodeDataAndTransferToGlobalMemory<T>(MpiEnvironment environment, Func<int, T> calcNodeData);

		Dictionary<int, T> CalcNodeDataAndTransferToLocalMemory<T>(
			MpiEnvironment environment, Func<int, T> calcNodeData);

		void DoGlobalOperation(MpiEnvironment environment, Action globalOperation);
	}
}
