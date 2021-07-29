using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace MGroup.Environments
{
	/// <summary>
	/// Operations per each <see cref="ComputeNode"/> are run sequentially. The data for all <see cref="ComputeNode"/>s are 
	/// assumed to exist in the same shared memory address space.
	/// </summary>
	public class SequentialSharedEnvironment : IComputeEnvironment
	{
		private ComputeNodeTopology nodeTopology;

		public SequentialSharedEnvironment()
		{
		}

		public bool AllReduceAnd(Dictionary<int, bool> valuePerNode)
		{
			bool result = true;
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				result &= valuePerNode[nodeID];
			}
			return result;
		}

		public double AllReduceSum(Dictionary<int, double> valuePerNode)
		{
			double sum = 0.0;
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				sum += valuePerNode[nodeID];
			}
			return sum;
		}

		public Dictionary<int, T> CreateDictionaryPerNode<T>(Func<int, T> createDataPerNode)
		{
			var result = new Dictionary<int, T>();
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				result[nodeID] = createDataPerNode(nodeID);
			}
			return result;
		}

		public void DoGlobalOperation(Action globalOperation)
		{
			globalOperation();
		}

		public void DoPerNode(Action<int> actionPerNode)
		{
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				actionPerNode(nodeID);
			}
		}

		public ComputeNode GetComputeNode(int nodeID) => nodeTopology.Nodes[nodeID];

		public void Initialize(ComputeNodeTopology nodeTopology)
		{
			this.nodeTopology = nodeTopology;
		}

		public void NeighborhoodAllToAll<T>(Dictionary<int, AllToAllNodeData<T>> dataPerNode, bool areRecvBuffersKnown)
		{
			foreach (int thisNodeID in nodeTopology.Nodes.Keys)
			{
				ComputeNode thisNode = nodeTopology.Nodes[thisNodeID];
				AllToAllNodeData<T> thisData = dataPerNode[thisNodeID];

				foreach (int otherNodeID in thisNode.Neighbors)
				{
					// Receive data from each other node, by just copying the corresponding array segments.
					ComputeNode otherNode = nodeTopology.Nodes[otherNodeID];
					AllToAllNodeData<T> otherData = dataPerNode[otherNodeID];
					int bufferLength = otherData.sendValues[thisNodeID].Length;

					if (!areRecvBuffersKnown)
					{
						Debug.Assert(!thisData.recvValues.ContainsKey(otherNodeID), "This buffer must not exist previously.");
						thisData.recvValues[otherNodeID] = new T[bufferLength];
					}
					else
					{
						Debug.Assert(thisData.recvValues[otherNodeID].Length == bufferLength,
							$"Node {otherNode.ID} tries to send {bufferLength} entries but node {thisNode.ID} tries to" +
								$" receive {thisData.recvValues[otherNodeID].Length} entries. They must match.");
					}

					// Copy data from other to this node. 
					// Copying from this to other node will be done in another iteration of the outer loop.
					Array.Copy(otherData.sendValues[thisNodeID], thisData.recvValues[otherNodeID], bufferLength);
				}
			}
		}

		public Dictionary<int, T> TransferNodeDataToGlobalMemory<T>(Func<int, T> getLocalNodeData)
		{
			var result = new Dictionary<int, T>();
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				result[nodeID] = getLocalNodeData(nodeID);
			}
			return result;
		}

		public Dictionary<int, T> TransferNodeDataToLocalMemories<T>(Dictionary<int, T> globalNodeDataStorage)
			=> new Dictionary<int, T>(globalNodeDataStorage);
	}
}
