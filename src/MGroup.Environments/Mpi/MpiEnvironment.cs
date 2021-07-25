using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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
		private readonly MpiNet.Intracommunicator commWorld;
		private readonly MpiNet.Environment mpiEnvironment;

		private bool disposed = false;
		private Dictionary<int, ComputeNode> localNodes;
		private ComputeNodeTopology nodeTopology;
		private MpiP2PTransfers p2pTransfers;

		public MpiEnvironment()
		{
			//TODOMPI: See Threading param. In multithreaded programs, I must specify that to MPI.NET.
			string[] args = Array.Empty<string>();
			this.mpiEnvironment = new MpiNet.Environment(ref args);
			this.commWorld = MpiNet.Communicator.world;
		}

		~MpiEnvironment()
		{
			Dispose(false);
		}

		public bool AllReduceAnd(Dictionary<int, bool> valuePerNode)
		{
			bool localValue = true;
			foreach (int nodeID in localNodes.Keys)
			{
				localValue &= valuePerNode[nodeID];
			}
			return commWorld.Allreduce(localValue, MpiNet.Operation<bool>.LogicalAnd);
		}

		public double AllReduceSum(Dictionary<int, double> valuePerNode)
		{
			//TODOMPI: reductions for local nodes can be done more efficiently. See TplSharedEnvironment
			double localValue = 0.0;
			foreach (int nodeID in localNodes.Keys) 
			{
				localValue += valuePerNode[nodeID];
			}
			return commWorld.Allreduce(localValue, MpiNet.Operation<double>.Add);
		}

		public Dictionary<int, T> CreateDictionaryPerNode<T>(Func<int, T> createDataPerNode)
		{
			// Add the keys first to avoid race conditions
			var result = new Dictionary<int, T>();
			foreach (int nodeID in localNodes.Keys)
			{
				result[nodeID] = default;
			}

			// Run the operation per local node in parallel and store the individual results.
			Parallel.ForEach(localNodes.Keys, nodeID => result[nodeID] = createDataPerNode(nodeID));

			return result;
		}

		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}

		public void DoPerNode(Action<int> actionPerNode)
		{
			Parallel.ForEach(localNodes.Keys, actionPerNode);
		}

		public void DoSingle(Action action)
		{
			if (commWorld.Rank == 0)
			{
				action();
			}
		}

		//TODOMPI: Catch KeyNotFoundException and throw a custom RemoteProcessDataAccessException (or something similar). 
		//      First check if localNodes is initialized.
		public ComputeNode GetComputeNode(int nodeID) => localNodes[nodeID];

		public void Initialize(ComputeNodeTopology nodeTopology)
		{
			//TODOMPI: Perhaps this validation is useful for more than just the MpiEnvironment and should be done elsewhere.
			// Check cluster IDs
			if (nodeTopology.Clusters.Count != commWorld.Size)
			{
				throw new ArgumentException(
					$"There number of compute node clusters ({nodeTopology.Clusters.Count}) does not match"
					+ $" the actual number of processes launched ({commWorld.Size})");
			}
			for (int p = 0; p < commWorld.Size; ++p)
			{
				if (!nodeTopology.Clusters.ContainsKey(p))
				{
					throw new ArgumentException("Cluster IDs must be numbered contiguously in [0, numClusters)");
				}
			}

			// Check that each cluster contains at least 1 node
			foreach (ComputeNodeCluster cluster in nodeTopology.Clusters.Values)
			{
				if (cluster.Nodes.Count < 1)
				{
					throw new ArgumentException(
						$"Each cluster most contain at least 1 compute node, but cluster {cluster.ID} contains none.");
				}
			}

			// Store the topology and nodes belonging to the cluster with the same ID as this MPI process.
			ComputeNodeCluster localCluster = nodeTopology.Clusters[commWorld.Rank];
			this.localNodes = new Dictionary<int, ComputeNode>(localCluster.Nodes);
			this.nodeTopology = nodeTopology;

			// Analyze local and remote communication cases
			this.p2pTransfers = new MpiP2PTransfers(nodeTopology, localCluster);
		}

		public void NeighborhoodAllToAll<T>(Dictionary<int, AllToAllNodeData<T>> dataPerNode, bool areRecvBuffersKnown)
		{
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
						ComputeNodeCluster remoteCluster = nodeTopology.Nodes[neighborID].Cluster;
						int remoteProcess = remoteCluster.ID;
						int tag = p2pTransfers.GetSendRecvTag(
							MpiJob.TransferBufferLengthDuringNeighborhoodAllToAll, node.ID, neighborID);

						Action<int> allocateBuffer = length => data.recvValues[neighborID] = new T[length];
						recvLengthRequests.Add(commWorld.ImmediateReceive<int>(remoteProcess, tag, allocateBuffer));

						int bufferLength = data.sendValues[neighborID].Length;
						sendLengthRequests.Add(commWorld.ImmediateSend(bufferLength, remoteProcess, tag));
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
					ComputeNodeCluster remoteCluster = nodeTopology.Nodes[neighborID].Cluster;
					int remoteProcess = remoteCluster.ID;
					int tag = p2pTransfers.GetSendRecvTag(MpiJob.TransferBufferDuringNeighborhoodAllToAll, node.ID, neighborID);

					T[] recvBuffer = data.recvValues[neighborID];
					recvRequests.Add(commWorld.ImmediateReceive(remoteProcess, tag, recvBuffer));

					T[] sendBuffer = data.sendValues[neighborID];
					sendRequests.Add(commWorld.ImmediateSend(sendBuffer, remoteProcess, tag));
				}
			}

			// Transfer buffers between local nodes, while waiting for the posted MPI requests.
			Action<int> transferLocalBuffers = thisNodeID =>
			{
				AllToAllNodeData<T> thisData = dataPerNode[thisNodeID];

				foreach (int neighborID in p2pTransfers.GetLocalNeighborsOf(thisNodeID))
				{
					// Receive data from each other node, by just copying the corresponding array segments.
					ComputeNode otherNode = nodeTopology.Nodes[neighborID];
					AllToAllNodeData<T> otherData = dataPerNode[neighborID];
					int bufferLength = otherData.sendValues[thisNodeID].Length;

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
					Array.Copy(otherData.sendValues[thisNodeID], thisData.recvValues[neighborID], bufferLength);
				}
			};
			Parallel.ForEach(localNodes.Keys, transferLocalBuffers);

			// Wait for MPI requests to end 
			recvRequests.WaitAll();
			sendRequests.WaitAll();
		}

		private void Dispose(bool disposing)
		{
			if (!disposed)
			{
				if (disposing)
				{
					// DO NOT DISPOSE Communicator.world here, since it is not owned by this class.
					// DISPOSE other communicators here, e.g. GraphCommunicator for neighborhoods.

					if ((mpiEnvironment != null) && (MpiNet.Environment.Finalized == false))
					{
						mpiEnvironment.Dispose();
					}
				}

				// If there were unmanaged resources, they should be disposed here
			}
			disposed = true;
		}
	}
}
