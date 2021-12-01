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

		public bool AllReduceOr(IDictionary<int, bool> valuePerNode)
		{
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				if (valuePerNode[nodeID])
				{
					return true;
				}
			}
			return false;
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

		public double[] AllReduceSum(int numReducedValues, Dictionary<int, double[]> valuesPerNode)
		{
			var sum = new double[numReducedValues];
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				double[] nodeValues = valuesPerNode[nodeID];
				for (int i = 0; i < numReducedValues; ++i)
				{
					sum[i] += nodeValues[i];
				}
			}
			return sum;
		}

		public Dictionary<int, T> CalcNodeData<T>(Func<int, T> calcNodeData)
		{
			var result = new Dictionary<int, T>(nodeTopology.Nodes.Count);
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				result[nodeID] = calcNodeData(nodeID);
			}
			return result;
		}

		public Dictionary<int, T> CalcNodeDataAndTransferToGlobalMemory<T>(Func<int, T> calcNodeData)
			=> CalcNodeData(calcNodeData);

		public Dictionary<int, T> CalcNodeDataAndTransferToGlobalMemoryPartial<T>(Func<int, T> calcNodeData, 
			Func<int, bool> isActiveNode)
		{
			var result = new Dictionary<int, T>(nodeTopology.Nodes.Count);
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				if (isActiveNode(nodeID))
				{
					result[nodeID] = calcNodeData(nodeID);
				}
			}
			return result;
		}

		public Dictionary<int, T> CalcNodeDataAndTransferToLocalMemory<T>(Func<int, T> calcNodeData)
			=> CalcNodeData(calcNodeData);

		public void DoGlobalOperation(Action globalOperation)
		{
			globalOperation();
		}

		public Dictionary<int, T> DoPerItemInGlobalMemory<T>(IEnumerable<int> items, Func<int, T> calcItemData)
		{
			var result = new Dictionary<int, T>();
			foreach (int item in items)
			{
				result[item] = calcItemData(item);
			}
			return result;
		}

		public void DoPerNode(Action<int> actionPerNode)
		{
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				actionPerNode(nodeID);
			}
		}

		public void DoPerNodeSerially(Action<int> actionPerNode) => DoPerNode(actionPerNode);

		public ComputeNode GetComputeNode(int nodeID) => nodeTopology.Nodes[nodeID];

		public void Initialize(ComputeNodeTopology nodeTopology)
		{
			this.nodeTopology = nodeTopology;
		}

		public void NeighborhoodAllToAll<T>(Dictionary<int, AllToAllNodeData<T>> dataPerNode, bool areRecvBuffersKnown)
		{
			CheckNeighborhoodAllToAllInput(dataPerNode, areRecvBuffersKnown);
			foreach (int thisNodeID in nodeTopology.Nodes.Keys)
			{
				ComputeNode thisNode = nodeTopology.Nodes[thisNodeID];
				AllToAllNodeData<T> thisData = dataPerNode[thisNodeID];

				foreach (int otherNodeID in thisNode.Neighbors)
				{
					// Receive data from each other node, by just copying the corresponding array segments.
					AllToAllNodeData<T> otherData = dataPerNode[otherNodeID];
					bool haveCommonData = otherData.sendValues.TryGetValue(thisNodeID, out T[] dataToSend);
					if (!haveCommonData)
					{
						continue;
					}

					int bufferLength = dataToSend.Length;
					if (!areRecvBuffersKnown)
					{
						thisData.recvValues[otherNodeID] = new T[bufferLength];
					}

					// Copy data from other to this node. 
					// Copying from this to other node will be done in another iteration of the outer loop.
					Array.Copy(dataToSend, thisData.recvValues[otherNodeID], bufferLength);
				}
			}
		}

		[Conditional("DEBUG")]
		private void CheckNeighborhoodAllToAllInput<T>(Dictionary<int, AllToAllNodeData<T>> dataPerNode, bool areRecvBuffersKnown)
		{
			foreach (int thisNodeID in nodeTopology.Nodes.Keys)
			{
				ComputeNode thisNode = nodeTopology.Nodes[thisNodeID];
				AllToAllNodeData<T> thisData = dataPerNode[thisNodeID];

				Debug.Assert(thisNode.Neighbors.IsSupersetOf(thisData.sendValues.Keys));
				Debug.Assert(thisNode.Neighbors.IsSupersetOf(thisData.recvValues.Keys));
				foreach (int otherNodeID in thisNode.Neighbors)
				{
					// Receive data from each other node, by just copying the corresponding array segments.
					ComputeNode otherNode = nodeTopology.Nodes[otherNodeID];
					AllToAllNodeData<T> otherData = dataPerNode[otherNodeID];

					// Check that the buffers exist and have the correct length.
					if (otherData.sendValues.ContainsKey(thisNodeID))
					{
						int bufferLength = otherData.sendValues[thisNodeID].Length;

						if (!areRecvBuffersKnown)
						{
							
							Debug.Assert(!thisData.recvValues.ContainsKey(otherNodeID), "This buffer must not exist previously.");
						}
						else
						{
							Debug.Assert(thisData.recvValues[otherNodeID].Length == bufferLength,
								$"Node {otherNode.ID} tries to send {bufferLength} entries but node {thisNode.ID} tries to" +
									$" receive {thisData.recvValues[otherNodeID].Length} entries. They must match.");
						}
					}
				}
			}
		}
	}
}
