using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MPI;
using MpiNet = MPI;

//WARNING: MPI.NET (on windows) does not support MPI calls from multiple threads. Therefore exposing send and receive methods
//        is very risky. Instead when exposing on collective operations, MpiEnvironment can make sure all MPI calls are funneled
//        through the same thread.
//TODOMPI: Dedicated unit tests for each method of the environment classes. MPI tutorials and code reference may contain examples:
//      E.g.: https://www.rookiehpc.com/mpi/docs/mpi_alltoallv.php, 
//      http://www.math-cs.gordon.edu/courses/cps343/presentations/MPI_Collective.pdf
//TODOMPI: Map processes to actual hardware nodes in an efficient manner. Then the whole program (linear algebra, DDM, 
//      model creation) must depend on this mapping.
namespace MGroup.Environments.Mpi
{
	/// <summary>
	/// There is only one <see cref="ComputeNode"/> per MPI process. There may be many processes per machine, but each has its 
	/// own memory address space and they can communicate only through the MPI library. Aside from sharing hardware resources, 
	/// the processes and <see cref="ComputeNode"/>s are in essence run on different machines. 
	/// The data for a given <see cref="ComputeNode"/> and its <see cref="ComputeSubnode"/>s are assumed to exist in the same 
	/// shared memory address space. The execution of operations per <see cref="ComputeSubnode"/> of the local 
	/// <see cref="ComputeNode"/> depends on the <see cref="ISubnodeEnvironment"/> used. Do not use 2 instances of 
	/// <see cref="MpiEnvironment"/> in the same process, especially concurrently, as it is possible that their MPI calls may
	/// mix up their tags resulting in the wrong data being received. For the same reason, <see cref="MpiEnvironment"/> is not 
	/// thread safe.
	/// </summary>
	/// <remarks>
	/// Implements the Dispose pattern: 
	/// https://www.codeproject.com/Articles/15360/Implementing-IDisposable-and-the-Dispose-Pattern-P
	/// </remarks>
	public sealed class MpiEnvironment : IComputeEnvironment, IDisposable
	{
		private readonly MpiNet.Environment mpiEnvironment;

		private bool disposed = false;
		private Dictionary<int, ComputeNode> localNodes;
		private MpiP2PTransfers p2pTransfers;
		private MpiCollectivesHelper collectivesHelperWorld;
		private MpiCollectivesHelper collectivesHelperNodes;

		public MpiEnvironment(IMpiGlobalOperationStrategy globalOperationStrategy = null)
		{
			if (globalOperationStrategy != null)
			{
				this.GlobalOperationStrategy = globalOperationStrategy;
			}
			else
			{
				this.GlobalOperationStrategy = new MasterSlavesGlobalOperationStrategy();
			}

			//TODOMPI: See Threading param. In multithreaded programs, I must specify that to MPI.NET.
			string[] args = Array.Empty<string>();
			this.mpiEnvironment = new MpiNet.Environment(ref args);
			this.CommWorld = MpiNet.Communicator.world;
		}

		~MpiEnvironment()
		{
			Dispose(false);
		}

		public Intracommunicator CommWorld { get; }

		public Intracommunicator CommNodes { get; private set; }

		//TODO: Thread access should be controlled
		//TODO: Make sure that all implementations are stateless, so that they can be changed irrespectively from the initializations of this object
		public IMpiGlobalOperationStrategy GlobalOperationStrategy { get; set; }

		public ComputeNodeTopology NodeTopology { get ; private set; }

		public Dictionary<int, T> AllGather<T>(Func<int, T> getDataPerNode)
		{
			if (CommNodes == null)
			{
				return null;
			}

			T[] localData = collectivesHelperNodes.LocalNodesDataToArray(CommNodes.Rank, getDataPerNode);
			int[] counts = collectivesHelperNodes.NodeCounts;

			//TODO Optim: have a cached buffer to write data into (use the overload with ref parameter)
			T[] allData = CommNodes.AllgatherFlattened(localData, counts); // All processes gather the data

			return collectivesHelperNodes.AllNodesDataToDictionary(allData);
		}

		public bool AllReduceAnd(Dictionary<int, bool> valuePerNode)
		{
			if (CommNodes != null)
			{
				bool localValue = true;
				foreach (int nodeID in localNodes.Keys)
				{
					localValue &= valuePerNode[nodeID];
				}
				return CommWorld.Allreduce(localValue, MpiNet.Operation<bool>.LogicalAnd);
			}
			else
			{
				return CommWorld.Allreduce(default(bool), MpiNet.Operation<bool>.LogicalAnd);
			}
		}

		public bool AllReduceOr(IDictionary<int, bool> valuePerNode)
		{
			if (CommNodes != null)
			{
				bool localValue = false;
				foreach (int nodeID in localNodes.Keys)
				{
					if (valuePerNode[nodeID])
					{
						localValue = true;
						break;
					}
				}
				return CommWorld.Allreduce(localValue, MpiNet.Operation<bool>.LogicalOr);
			}
			else
			{
				return CommWorld.Allreduce(default(bool), MpiNet.Operation<bool>.LogicalOr);
			}
		}

		public double AllReduceSum(Dictionary<int, double> valuePerNode)
		{
			if (CommNodes != null)
			{
				//TODOMPI: reductions for local nodes can be done more efficiently. See TplSharedEnvironment
				double localValue = 0.0;
				foreach (int nodeID in localNodes.Keys)
				{
					localValue += valuePerNode[nodeID];
				}
				return CommWorld.Allreduce(localValue, MpiNet.Operation<double>.Add);
			}
			else
			{
				return CommWorld.Allreduce(default(double), MpiNet.Operation<double>.Add);
			}
		}

		public double[] AllReduceSum(int numReducedValues, Dictionary<int, double[]> valuesPerNode)
		{
			var localValues = new double[numReducedValues];
			if (CommNodes != null)
			{
				//TODOMPI: reductions for local nodes can be done more efficiently. See TplSharedEnvironment
				foreach (int nodeID in localNodes.Keys)
				{
					double[] nodeValues = valuesPerNode[nodeID];
					for (int i = 0; i < numReducedValues; ++i)
					{
						localValues[i] += nodeValues[i];
					}
				}
			}
			return CommWorld.Allreduce(localValues, MpiNet.Operation<double>.Add);
		}

		public Dictionary<int, T> CalcNodeData<T>(Func<int, T> calcNodeData)
		{
			if (CommNodes == null)
			{
				return null;
			}

			// Add the keys first to avoid race conditions
			var result = new Dictionary<int, T>(localNodes.Count);
			foreach (int nodeID in localNodes.Keys)
			{
				result[nodeID] = default;
			}

			// Run the operation per local node in parallel and store the individual results.
			Parallel.ForEach(localNodes.Keys, nodeID => result[nodeID] = calcNodeData(nodeID));
			return result;
		}

		public Dictionary<int, T> CalcNodeDataAndTransferToGlobalMemory<T>(Func<int, T> calcNodeData)
			=> GlobalOperationStrategy.CalcNodeDataAndTransferToGlobalMemory(this, calcNodeData);

		public Dictionary<int, T> CalcNodeDataAndTransferToGlobalMemoryPartial<T>(Func<int, T> calcNodeData,
			Func<int, bool> isActiveNode)
			=> throw new NotImplementedException("Perhaps I could call the non-partial method instead of throwing exceptions");

		public Dictionary<int, T> CalcNodeDataAndTransferToLocalMemory<T>(Func<int, T> subdomainOperation)
			=> GlobalOperationStrategy.CalcNodeDataAndTransferToLocalMemory(this, subdomainOperation);

		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}

		public void DoGlobalOperation(Action globalOperation) 
			=> GlobalOperationStrategy.DoGlobalOperation(this, globalOperation);

		public Dictionary<int, T> DoPerItemInGlobalMemory<T>(IEnumerable<int> items, Func<int, T> calcItemData)
		{
			Dictionary<int, T> result = null;

			//TODOMPI: This is not necessary. The whole method should be called inside a DoGlobalOperation(), thus only for
			//	the processes that are responsible for global operations
			//globalOperationStrategy.DoGlobalOperation(this, () =>
			//{
			//	result = new Dictionary<int, T>();
			//	foreach (int item in items)
			//	{
			//		result[item] = default;
			//	}
			//	Parallel.ForEach(items, x => result[x] = calcItemData(x));
			//});
			result = new Dictionary<int, T>();
			foreach (int item in items)
			{
				result[item] = default;
			}
			Parallel.ForEach(items, x => result[x] = calcItemData(x));

			return result;
		}

		public void DoPerNode(Action<int> actionPerNode)
		{
			if (CommNodes == null)
			{
				return;
			}

			Parallel.ForEach(localNodes.Keys, actionPerNode);
		}

		public void DoPerNodeSerially(Action<int> actionPerNode)
		{
			if (CommNodes == null)
			{
				return;
			}

			foreach (int nodeID in localNodes.Keys)
			{
				actionPerNode(nodeID);
			}
		}

		public Dictionary<int, T> GatherToRootProcess<T>(Func<int, T> getDataPerNode, int rootProcessRank)
		{
			T[] localData;
			if (CommNodes == null)
			{
				localData = new T[0];
			}
			else
			{
				localData = collectivesHelperWorld.LocalNodesDataToArray(CommWorld.Rank, getDataPerNode);
			}

			int[] counts = collectivesHelperWorld.NodeCounts;
			if (CommWorld.Rank == rootProcessRank) // Root process gathers data
			{
				//TODO Optim: have a cached buffer to write data into (use the overload with ref parameter)
				T[] allData = CommWorld.GatherFlattened(localData, counts, rootProcessRank);
				return collectivesHelperWorld.AllNodesDataToDictionary(allData);

			}
			else // Other processes send data to root process
			{
				CommWorld.GatherFlattened(localData, counts, rootProcessRank);
				return null;

			}
		}

		//TODOMPI: Catch KeyNotFoundException and throw a custom RemoteProcessDataAccessException (or something similar). 
		//      First check if localNodes is initialized.
		public ComputeNode GetComputeNode(int nodeID) => localNodes[nodeID];

		public void Initialize(ComputeNodeTopology nodeTopology)
		{
			//TODO: First of all, clear all existing data

			nodeTopology.CheckSanity();

			//TODOMPI: Perhaps this validation is useful for more than just the MpiEnvironment and should be done elsewhere.
			// Check cluster IDs
			if (nodeTopology.Clusters.Count > CommWorld.Size)
			{
				throw new ArgumentException(
					$"The number of compute node clusters ({nodeTopology.Clusters.Count}) exceeds"
					+ $" the actual number of processes launched ({CommWorld.Size})");
			}

			// Store the topology and nodes belonging to the cluster with the same ID as this MPI process.
			this.NodeTopology = nodeTopology;
			if (CommWorld.Rank < nodeTopology.Clusters.Count) // Only for processes that accommodate compute nodes 
			{
				ComputeNodeCluster localCluster = nodeTopology.Clusters[CommWorld.Rank];
				this.localNodes = new Dictionary<int, ComputeNode>(localCluster.Nodes);

				// Analyze local and remote communication cases between nodes
				this.p2pTransfers = new MpiP2PTransfers(nodeTopology, localCluster);
			}

			// Create a subcommunicator only for processes that accommodate compute nodes
			if (CommNodes != null)
			{
				CommNodes.Dispose();
				CommNodes = null;
			}
			int[] processesWithNodes = Enumerable.Range(0, nodeTopology.Clusters.Count).ToArray();
			CommNodes =(Intracommunicator)CommWorld.Create(CommWorld.Group.IncludeOnly(processesWithNodes));
			Debug.Assert((CommNodes != null) == (CommWorld.Rank < nodeTopology.Clusters.Count));

			// Prevaluate data used in collective communications
			int numExtraProcesses = CommWorld.Size - nodeTopology.Clusters.Count;
			this.collectivesHelperWorld = new MpiCollectivesHelper(nodeTopology, numExtraProcesses);
			this.collectivesHelperNodes = new MpiCollectivesHelper(nodeTopology, 0);
		}

		public void NeighborhoodAllToAll<T>(Dictionary<int, AllToAllNodeData<T>> dataPerNode, bool areRecvBuffersKnown)
		{
			if (CommNodes == null)
			{
				return;
			}

			//TODOMPI: This can be improved greatly. As soon as a process receives the length of its recv buffer, the actual data 
			//      can be transfered between these 2 processes. There is no need to wait for the other p2p length communications.

			// Transfer buffer lengths to/from remote nodes, via non-blocking send/receive operations.
			// MPI.NET does not support posting MPI requests from multiple threads at once. 
			// However all these requests are non-blocking, thus posting them serially is efficient.
			if (!areRecvBuffersKnown)
			{
				var recvLengthRequests = new MpiNet.RequestList(); //TODOMPI: Use my own RequestList implementation for more efficient WaitAll() and pipeline opportunities
				var sendLengthRequests = new MpiNet.RequestList(); //TODOMPI: Can't I avoid waiting for send requests? Especially for the lengths
				foreach (ComputeNode node in localNodes.Values)
				{
					AllToAllNodeData<T> data = dataPerNode[node.ID];
					foreach (int neighborID in p2pTransfers.GetRemoteNeighborsOf(node.ID))
					{
						ComputeNodeCluster remoteCluster = NodeTopology.Nodes[neighborID].Cluster;
						int remoteProcess = remoteCluster.ID;
						int tag = p2pTransfers.GetSendRecvTag(
							MpiJob.TransferBufferLengthDuringNeighborhoodAllToAll, node.ID, neighborID);

						Action<int> allocateBuffer = length => data.recvValues[neighborID] = new T[length];
						recvLengthRequests.Add(CommNodes.ImmediateReceive<int>(remoteProcess, tag, allocateBuffer));

						int bufferLength;
						bool mustSend = data.sendValues.TryGetValue(neighborID, out T[] sendBuffer);
						if (mustSend)
						{
							// This includes the case sendBuffer.Length == 0
							bufferLength = sendBuffer.Length;
						}
						else
						{
							bufferLength = 0;
						}
						sendLengthRequests.Add(CommNodes.ImmediateSend(bufferLength, remoteProcess, tag));
					}
				}

				// Wait for requests to end.
				//TODOMPI: For now this serves to prevent receiving a buffer without first receiving its length.
				//      However this implementation amounts to having a neighborhood-level barrier. 
				//      A better one would be to use dependent tasks.
				recvLengthRequests.WaitAll();
				sendLengthRequests.WaitAll();
			}

			// Transfer buffers to/from remote nodes, via non-blocking send/receive operations.
			// MPI.NET does not support posting MPI requests from multiple threads at once. 
			// However all these requests are non-blocking, thus posting them serially is efficient.
			var recvRequests = new MpiNet.RequestList(); //TODOMPI: Use my own RequestList implementation for more efficient WaitAll() and pipeline opportunities
			var sendRequests = new MpiNet.RequestList(); //TODOMPI: Can't I avoid waiting for send requests? 
			foreach (ComputeNode node in localNodes.Values)
			{
				AllToAllNodeData<T> data = dataPerNode[node.ID];
				
				foreach (int neighborID in p2pTransfers.GetRemoteNeighborsOf(node.ID))
				{
					ComputeNodeCluster remoteCluster = NodeTopology.Nodes[neighborID].Cluster;
					int remoteProcess = remoteCluster.ID;
					int tag = p2pTransfers.GetSendRecvTag(MpiJob.TransferBufferDuringNeighborhoodAllToAll, node.ID, neighborID);



					bool mustSend = data.sendValues.TryGetValue(neighborID, out T[] sendBuffer);
					if (mustSend && sendBuffer.Length > 0)
					{
						sendRequests.Add(CommNodes.ImmediateSend(sendBuffer, remoteProcess, tag));
					}

					bool mustRecv = data.recvValues.TryGetValue(neighborID, out T[] recvBuffer);
					if (mustRecv && recvBuffer.Length > 0)
					{
						recvRequests.Add(CommNodes.ImmediateReceive(remoteProcess, tag, recvBuffer));
					}

					// Not sure about this
					//Debug.Assert((mustSend == mustRecv) && (sendBuffer.Length == recvBuffer.Length));
				}
			}

			// Transfer buffers between local nodes, while waiting for the posted MPI requests.
			Action<int> transferLocalBuffers = thisNodeID =>
			{
				AllToAllNodeData<T> thisData = dataPerNode[thisNodeID];

				foreach (int neighborID in p2pTransfers.GetLocalNeighborsOf(thisNodeID))
				{
					// Receive data from each other node, by just copying the corresponding array segments.
					ComputeNode otherNode = NodeTopology.Nodes[neighborID];
					AllToAllNodeData<T> otherData = dataPerNode[neighborID];

					bool haveCommonData = otherData.sendValues.TryGetValue(thisNodeID, out T[] dataToSend);
					if (!haveCommonData)
					{
						continue;
					}

					int bufferLength = dataToSend.Length;
					if (!areRecvBuffersKnown)
					{
						Debug.Assert(!thisData.recvValues.ContainsKey(neighborID), "This buffer must not exist previously.");
						thisData.recvValues[neighborID] = new T[bufferLength];
					}
					else
					{
						Debug.Assert(thisData.recvValues[neighborID].Length == bufferLength,
							$"Node {otherNode.ID} tries to send {bufferLength} entries but node {thisNodeID} tries to" +
								$" receive {thisData.recvValues[neighborID].Length} entries. They must match.");
					}

					// Copy data from other to this node. 
					// Copying from this to other node will be done in another iteration of the outer loop.
					Array.Copy(dataToSend, thisData.recvValues[neighborID], bufferLength);
				}
			};
			Parallel.ForEach(localNodes.Keys, transferLocalBuffers);

			// Wait for MPI requests to end 
			recvRequests.WaitAll();
			sendRequests.WaitAll();
		}

		public Dictionary<int, T> ScatterFromRootProcess<T>(Dictionary<int, T> dataPerNode, int rootProcessRank)
		{
			T[] localData;
			int[] counts = collectivesHelperWorld.NodeCounts;
			if (CommWorld.Rank == rootProcessRank) // Master process gathers data
			{
				T[] allData = collectivesHelperWorld.AllNodesDataToArray(dataPerNode);

				//TODO Optim: have a cached buffer to write data into (use the overload with ref parameter)
				localData = CommWorld.ScatterFromFlattened(allData, counts, rootProcessRank);
			}
			else // Slave processes send data to master process
			{
				localData = CommWorld.ScatterFromFlattened(new T[0], counts, rootProcessRank);
			}

			if (CommNodes == null)
			{
				return null;
			}
			else
			{
				return collectivesHelperWorld.LocalNodesDataToDictionary(CommWorld.Rank, localData);
			}
		}

		private void Dispose(bool disposing)
		{
			if (!disposed)
			{
				if (disposing)
				{
					// DO NOT DISPOSE Communicator.world here, since it is not owned by this class.
					// DISPOSE other communicators here, e.g. GraphCommunicator for neighborhoods.
					if (CommNodes != null)
					{
						CommNodes.Dispose();
					}

					if ((mpiEnvironment != null) && (MpiNet.Environment.Finalized == false))
					{
						mpiEnvironment.Dispose();
					}
				}

				// If there were unmanaged resources, they should be disposed here
			}
			disposed = true;
		}

		private MpiCollectivesHelper FindCollectivesHelper(Intracommunicator comm)
		{
			if (comm == CommNodes)
			{
				return collectivesHelperNodes;
			}
			else if (comm == CommWorld)
			{
				return collectivesHelperWorld;
			}
			else
			{
				throw new ArgumentException("Incompatible communicator");
			}
		}
	}
}
