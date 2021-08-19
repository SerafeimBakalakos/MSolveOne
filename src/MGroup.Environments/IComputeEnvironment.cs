using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Text;

//TODOMPI: Use Func<int, T> instead of Dictionary<int, T> as parameter. It will also allow lazy calculation fo subdomain data.
//      If I need the semantics of "already calculated data are passed into the environment" then I could have extension methods
//      that use Dictionaries and call IComputeEnvironment methods, 
//      e.g. Reduce(this IComputeEnvironment environment, Dictionary<int, double> values) => environment.Reduce(n => values[n])
namespace MGroup.Environments
{
	/// <summary>
	/// Manages a collection of compute nodes (e.g. MPI processes, C# threads, etc). Each compute node has its own distributed 
	/// memory, even if all nodes are run on the same CPU thread. As such, classes that implement this interface, describe 
	/// execution of operations across nodes (parallel, sequential, etc), data transfer between each node's memory and 
	/// synchronization of the nodes.
	/// </summary>
	public interface IComputeEnvironment
	{
		bool AllReduceAnd(Dictionary<int, bool> valuePerNode);

		bool AllReduceOr(IDictionary<int, bool> valuePerNode);

		double AllReduceSum(Dictionary<int, double> valuePerNode);

		double[] AllReduceSum(int numReducedValues, Dictionary<int, double[]> valuesPerNode);

		/// <summary>
		/// Performs <paramref name="calcNodeData"/> on each <see cref="ComputeNode"/> and returns a dictionary where 
		/// keys are the ids of the nodes. 
		/// <paramref name="calcNodeData"/> will be run on the memory space where each node is accommodated. 
		/// Similarly, the resulting dictionary in each memory space will contain data for the nodes that are accommodated there. 
		/// </summary>
		/// <typeparam name="T"></typeparam>
		/// <param name="calcNodeData"></param>
		Dictionary<int, T> CalcNodeData<T>(Func<int, T> calcNodeData);

		/// <summary>
		/// Performs <paramref name="calcNodeData"/> on each <see cref="ComputeNode"/> and returns a dictionary where 
		/// keys are the ids of the nodes. <paramref name="calcNodeData"/> will be run on the memory space where each node
		/// is accomodated. 
		/// Then the data for each node will be transfered to the memory space that is assigned to global operations and placed
		/// on the resulting dictionary. In all other memory spaces, null will be returned.
		/// </summary>
		/// <typeparam name="T"></typeparam>
		/// <param name="calcNodeData"></param>
		Dictionary<int, T> CalcNodeDataAndTransferToGlobalMemory<T>(Func<int, T> calcNodeData);

		/// <summary>
		/// Same as <see cref="CalcNodeDataAndTransferToGlobalMemory{T}(Func{int, T})"/>, but only for nodes that satisfy 
		/// <paramref name="isActiveNode"/>.
		/// </summary>
		Dictionary<int, T> CalcNodeDataAndTransferToGlobalMemoryPartial<T>(Func<int, T> calcNodeData, 
			Func<int, bool> isActiveNode);

		/// <summary>
		/// Performs <paramref name="calcNodeData"/> on each <see cref="ComputeNode"/> and returns a dictionary where 
		/// keys are the ids of the nodes. 
		/// <paramref name="calcNodeData"/> will be run only on the memory space that is assigned to global operations. 
		/// On all other memory spaces, it will be ignored. 
		/// Then the data for each node will be transfered to the memory space that accommodates that node and placed in the
		/// resulting dictionary that exists in that memory space. 
		/// </summary>
		/// <typeparam name="T"></typeparam>
		/// <param name="calcNodeData"></param>
		Dictionary<int, T> CalcNodeDataAndTransferToLocalMemory<T>(Func<int, T> calcNodeData);

		/// <summary>
		/// Executes <paramref name="globalOperation"/> only on the memory space that is assigned to global operations.
		/// </summary>
		/// <param name="globalOperation"></param>
		void DoGlobalOperation(Action globalOperation);

		/// <summary>
		/// Executes <paramref name="actionPerNode"/> for each node on the memory space where that node is accommodated.
		/// </summary>
		/// <param name="actionPerNode"></param>
		void DoPerNode(Action<int> actionPerNode);

		//TODOMPI: Its most common use is weird: An Action<int> is called by the environment. The environment passes the id of 
		//      each ComputeNode it manages. Then the Action<int> requests from the environment to provide the ComputeNode for
		//      the same id that the environment provided, e.g. to inspect the neighboring ComputeNode ids.
		ComputeNode GetComputeNode(int nodeID);

		//TODOMPI: This should probably be done in the ctor. However in the current design the topology is identified by the 
		//      classes that are injected with IComputeEnvironment in their ctor. Having this method provides the (unwanted?)
		//      benefit of being able to change the topology later. This might come in handy in tests, since disposing the 
		//      underlying MPI classes, makes it very difficult if not impossible to reuse them. 
		/// <summary>
		/// Initializes the environment (e.g. works out communication paths). This must be always called exactly once, and 
		/// before any other member.
		/// </summary>
		void Initialize(ComputeNodeTopology nodeTopology); 

		//TODOMPI: Overload that uses delegates for assembling the send data and processing the receive data per neighbor of 
		//      each compute node. This will result in better pipelining, which I think will greatly improve performance and 
		//      essentially hide the communication cost, considering that clients generally do a lot of computations. 
		//TODOMPI: Alternatively expose non-blocking send and receive operations, to clients so that
		//      they can do them themselves. These may actualy help to avoid unnecessary buffers in communications between
		//      local nodes, for even greater benefit. However it forces clients to mess with async code.
		//      Perhaps IComputeEnvironment could facilitate the clients in their async code, by exposing ISend/IRecv that take
		//      delegates for creating the data (before send) and processing them (after recv) and by helping them to ensure 
		//      termination per node.
		void NeighborhoodAllToAll<T>(Dictionary<int, AllToAllNodeData<T>> dataPerNode, bool areRecvBuffersKnown);
	}

	//TODOMPI: Clients are forced to initialize sendValues and recvValues right now, which means client code is coupled with 
	//      knowledge that these dictionaries will be used concurrently.
	public class AllToAllNodeData<T>
	{
		/// <summary>
		/// Buffer of values that will be received by a <see cref="ComputeNode"/> i by each of its neighboring 
		/// <see cref="ComputeNode"/>s. Foreach j in <see cref="ComputeNode.Neighbors"/> of i, the values transfered from j to i 
		/// will be stored in <see cref="recvValues"/>[j]. If the buffer lengths are not known, then 
		/// </summary>
		public ConcurrentDictionary<int, T[]> recvValues;

		/// Buffer of values that will be sent from a <see cref="ComputeNode"/> i to each of its neighboring 
		/// <see cref="ComputeNode"/>s. Foreach j in <see cref="ComputeNode.Neighbors"/> of i, the values transfered from i to j 
		/// will be stored in <see cref="sendValues"/>[j]. 
		/// </summary>
		public ConcurrentDictionary<int, T[]> sendValues;
	}
}
