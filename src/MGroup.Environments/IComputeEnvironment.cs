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

		double AllReduceSum(Dictionary<int, double> valuePerNode);

		/// <summary>
		/// Keys are the ids of the <see cref="ComputeNode"/> objects managed by this environment.
		/// </summary>
		/// <typeparam name="T"></typeparam>
		/// <param name="createPerNode"></param>
		Dictionary<int, T> CreateDictionaryPerNode<T>(Func<int, T> createDataPerNode);

		void DoGlobalOperation(Action globalOperation);

		//Tout DoGlobalOperation<Tin, Tout>(Dictionary<int, Tin> nodeDataInGlobalMemory, Func<Dictionary<int, Tin>, Tout> globalOperation);

		//Dictionary<int, Tout> DoGlobalOperation<Tin, Tout>(Tin globalDataInGlobalMemory, Func<Tin, Dictionary<int, Tout>> globalOperation);

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

		/// <summary>
		/// Transfers node data from local memory spaces to global memory space.
		/// Input: In each memory space: data for each <see cref="ComputeNode"/> that is local to the memory space. 
		/// Output: In global memory space, data for all <see cref="ComputeNode"/>s will be returned. 
		/// In all other memory spaces, null will be returned.
		/// </summary>
		/// <typeparam name="T"></typeparam>
		/// <param name="getLocalNodeData"></param>
		Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(Func<int, T> getLocalNodeData);

		/// <summary>
		/// Transfers node data from global memory space to local memory spaces.
		/// Input: In global memory space, data for all <see cref="ComputeNode"/>s. In all other memory spaces, input will be 
		/// ignored.
		/// Output: In each memory space: data for each <see cref="ComputeNode"/> that is local to the memory space. 
		/// </summary>
		/// <typeparam name="T"></typeparam>
		/// <param name="globalNodeDataStorage">
		/// In the global memory address space, this containts the data for all nodes. In all other memory adress spaces, 
		/// it will be ignored.
		/// </param>
		Dictionary<int, T> TransferNodeDataToLocalMemories<T>(Dictionary<int, T> globalNodeDataStorage);
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

	//TODOMPI: Make these default interface methods
	//TODOMPI: I can also use overloaded GlobalOperation<> classes to hold the params and returned objects. It might be clearer 
	//		to describe each item there.
	public static class EnvironmentExtensions 
	{
		public static void DoGlobalOperation<Tin>(this IComputeEnvironment environment, 
			Func<int, Tin> getLocalNodeInput, Action<Dictionary<int, Tin>> globalOperation)
		{
			Dictionary<int, Tin> globalNodeInput = environment.TransferNodeDataToGlobalMemory(getLocalNodeInput);
			environment.DoGlobalOperation(() => globalOperation(globalNodeInput));
		}

		public static void DoGlobalOperation<Tin0, Tin1>(this IComputeEnvironment environment,
			Func<int, Tin0> getLocalNodeInput0, Func<int, Tin1> getLocalNodeInput1, 
			Action<Dictionary<int, Tin0>, Dictionary<int, Tin1>> globalOperation)
		{
			Dictionary<int, Tin0> globalNodeInput0 = environment.TransferNodeDataToGlobalMemory(getLocalNodeInput0);
			Dictionary<int, Tin1> globalNodeInput1 = environment.TransferNodeDataToGlobalMemory(getLocalNodeInput1);
			environment.DoGlobalOperation(() => globalOperation(globalNodeInput0, globalNodeInput1));
		}

		public static Dictionary<int, Tout> DoGlobalOperation<Tout>(this IComputeEnvironment environment,
			Func<Dictionary<int, Tout>> globalOperation)
		{
			Dictionary<int, Tout> globalNodeOutput = null;
			environment.DoGlobalOperation(() => globalNodeOutput = globalOperation());
			return environment.TransferNodeDataToLocalMemories(globalNodeOutput);
		}

		public static (Dictionary<int, Tout0> localNodeOutput0, Dictionary<int, Tout1> localNodeOutput1) 
			DoGlobalOperation<Tout0, Tout1>(this IComputeEnvironment environment,
			Func<(Dictionary<int, Tout0>, Dictionary<int, Tout1>)> globalOperation)
		{
			Dictionary<int, Tout0> globalNodeOutput0 = null;
			Dictionary<int, Tout1> globalNodeOutput1 = null;
			environment.DoGlobalOperation(() => (globalNodeOutput0, globalNodeOutput1) = globalOperation());
			Dictionary<int, Tout0> localNodeOutput0 = environment.TransferNodeDataToLocalMemories(globalNodeOutput0);
			Dictionary<int, Tout1> localNodeOutput1 = environment.TransferNodeDataToLocalMemories(globalNodeOutput1);
			return (localNodeOutput0, localNodeOutput1);
		}

		public static Dictionary<int, Tout> DoGlobalOperation<Tin, Tout>(this IComputeEnvironment environment,
			Func<int, Tin> getLocalNodeInput, Func<Dictionary<int, Tin>, Dictionary<int, Tout>> globalOperation)
		{
			Dictionary<int, Tin> globalNodeInput = environment.TransferNodeDataToGlobalMemory(getLocalNodeInput);
			Dictionary<int, Tout> globalNodeOutput = null;
			environment.DoGlobalOperation(() => globalNodeOutput = globalOperation(globalNodeInput));
			return environment.TransferNodeDataToLocalMemories(globalNodeOutput);
		}

		public static Dictionary<int, Tout> DoGlobalOperation<Tin0, Tin1, Tout>(this IComputeEnvironment environment,
			Func<int, Tin0> getLocalNodeInput0, Func<int, Tin1> getLocalNodeInput1, 
			Func<Dictionary<int, Tin0>, Dictionary<int, Tin1>, Dictionary<int, Tout>> globalOperation)
		{
			Dictionary<int, Tin0> globalNodeInput0 = environment.TransferNodeDataToGlobalMemory(getLocalNodeInput0);
			Dictionary<int, Tin1> globalNodeInput1 = environment.TransferNodeDataToGlobalMemory(getLocalNodeInput1);
			Dictionary<int, Tout> globalNodeOutput = null;
			environment.DoGlobalOperation(() => globalNodeOutput = globalOperation(globalNodeInput0, globalNodeInput1));
			return environment.TransferNodeDataToLocalMemories(globalNodeOutput);
		}

		public static (Dictionary<int, Tout0> localNodeOutput0, Dictionary<int, Tout1> localNodeOutput1) 
			DoGlobalOperation<Tin, Tout0, Tout1>(this IComputeEnvironment environment,
			Func<int, Tin> getLocalNodeInput, 
			Func<Dictionary<int, Tin>, (Dictionary<int, Tout0>, Dictionary<int, Tout1>)> globalOperation)
		{
			Dictionary<int, Tin> globalNodeInput = environment.TransferNodeDataToGlobalMemory(getLocalNodeInput);
			Dictionary<int, Tout0> globalNodeOutput0 = null;
			Dictionary<int, Tout1> globalNodeOutput1 = null;
			environment.DoGlobalOperation(() => (globalNodeOutput0, globalNodeOutput1) = globalOperation(globalNodeInput));
			Dictionary<int, Tout0> localNodeOutput0 = environment.TransferNodeDataToLocalMemories(globalNodeOutput0);
			Dictionary<int, Tout1> localNodeOutput1 = environment.TransferNodeDataToLocalMemories(globalNodeOutput1);
			return (localNodeOutput0, localNodeOutput1);
		}

		public static (Dictionary<int, Tout0> localNodeOutput0, Dictionary<int, Tout1> localNodeOutput1) 
			DoGlobalOperation<Tin0, Tin1, Tout0, Tout1>(this IComputeEnvironment environment,
			Func<int, Tin0> getLocalNodeInput0, Func<int, Tin1> getLocalNodeInput1,
			Func<Dictionary<int, Tin0>, Dictionary<int, Tin1>, (Dictionary<int, Tout0>, Dictionary<int, Tout1>)> globalOperation)
		{
			Dictionary<int, Tin0> globalNodeInput0 = environment.TransferNodeDataToGlobalMemory(getLocalNodeInput0);
			Dictionary<int, Tin1> globalNodeInput1 = environment.TransferNodeDataToGlobalMemory(getLocalNodeInput1);
			Dictionary<int, Tout0> globalNodeOutput0 = null;
			Dictionary<int, Tout1> globalNodeOutput1 = null;
			environment.DoGlobalOperation(
				() => (globalNodeOutput0, globalNodeOutput1) = globalOperation(globalNodeInput0, globalNodeInput1));
			Dictionary<int, Tout0> localNodeOutput0 = environment.TransferNodeDataToLocalMemories(globalNodeOutput0);
			Dictionary<int, Tout1> localNodeOutput1 = environment.TransferNodeDataToLocalMemories(globalNodeOutput1);
			return (localNodeOutput0, localNodeOutput1);
		}

	}
}
