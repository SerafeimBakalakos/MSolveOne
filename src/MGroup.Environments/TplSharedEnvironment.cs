using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Threading.Tasks;

namespace MGroup.Environments
{
	/// <summary>
	/// Operations per each <see cref="ComputeNode"/> are run concurrently, using the Task Parallel Library. 
	/// The data for all <see cref="ComputeNode"/>s are assumed to exist in the same shared memory address space.
	/// Clustering of <see cref="ComputeNode"/>s is ignored.
	/// </summary>
	public class TplSharedEnvironment : IComputeEnvironment
	{
		private readonly bool optimizeBuffers;
		private ComputeNodeTopology nodeTopology;

		public TplSharedEnvironment(bool optimizeBuffers = false)
		{
			this.optimizeBuffers = optimizeBuffers;
		}

		public Dictionary<int, T> AllGather<T>(Func<int, T> getDataPerNode) => CalcNodeData(getDataPerNode);

		public bool AllReduceAnd(Dictionary<int, bool> valuePerNode)
		{
			//TODOMPI: Reductions can be done more efficiently by having each thread reduce the values assigned to it. Then either
			//      reduce serially or with a binary tree (if a lot of threads are available)
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

		public int AllReduceSum(Func<int, int> calcNodeData)
		{
			int sum = 0;
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				sum += calcNodeData(nodeID);
			}
			return sum;
		}

		public double AllReduceSum(Dictionary<int, double> valuePerNode)
		{
			//TODOMPI: Reductions can be done more efficiently by having each thread reduce the values assigned to it. Then either
			//      reduce serially or with a binary tree (if a lot of threads are available)
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
			// Add the keys first to avoid race conditions
			var result = new Dictionary<int, T>(nodeTopology.Nodes.Count);
			foreach (int nodeID in nodeTopology.Nodes.Keys) 
			{
				result[nodeID] = default; 
			}

			// Run the operation per node in parallel and store the individual results.
			Parallel.ForEach(nodeTopology.Nodes.Keys, nodeID => result[nodeID] = calcNodeData(nodeID));

			return result;
		}

		public Dictionary<int, T> CalcNodeDataAndTransferToGlobalMemory<T>(Func<int, T> calcNodeData)
			=> CalcNodeData(calcNodeData);

		public Dictionary<int, T> CalcNodeDataAndTransferToGlobalMemoryPartial<T>(Func<int, T> calcNodeData,
			Func<int, bool> isActiveNode)
		{
			var sync = new object();
			var result = new Dictionary<int, T>(nodeTopology.Nodes.Count);
			foreach (int nodeID in nodeTopology.Nodes.Keys)
			{
				if (isActiveNode(nodeID))
				{
					T data = calcNodeData(nodeID);
					lock (sync)
					{
						result[nodeID] = data;
					}
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
				result[item] = default;
			}

			Parallel.ForEach(items, x => result[x] = calcItemData(x));

			return result;
		}

		public void DoPerNode(Action<int> actionPerNode)
		{
			Parallel.ForEach(nodeTopology.Nodes.Keys, actionPerNode);
		}

		public void DoPerNodeSerially(Action<int> actionPerNode)
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
			if (optimizeBuffers)
			{
				NeighborhoodAllToAllOptimized(dataPerNode, areRecvBuffersKnown);
			}
			else
			{
				NeighborhoodAllToAllGeneral(dataPerNode, areRecvBuffersKnown);
			}
		}

		private void NeighborhoodAllToAllGeneral<T>(Dictionary<int, AllToAllNodeData<T>> dataPerNode, bool areRecvBuffersKnown)
		{
			CheckNeighborhoodAllToAllInput(dataPerNode, areRecvBuffersKnown);
			Parallel.ForEach(nodeTopology.Nodes.Keys, thisNodeID =>
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
			});
		}

		private void NeighborhoodAllToAllOptimized<T>(Dictionary<int, AllToAllNodeData<T>> dataPerNode, bool areRecvBuffersKnown)
		{
			CheckNeighborhoodAllToAllInput(dataPerNode, areRecvBuffersKnown);
			Parallel.ForEach(nodeTopology.Nodes.Keys, thisNodeID =>
			{
				ComputeNode thisNode = nodeTopology.Nodes[thisNodeID];
				AllToAllNodeData<T> thisData = dataPerNode[thisNodeID];

				foreach (int otherNodeID in thisNode.Neighbors)
				{
					if (otherNodeID < thisNodeID)
					{
						continue; // We swap buffers once when thisNodeID < otherNodeID.
					}

					// Receive data from each other node.
					AllToAllNodeData<T> otherData = dataPerNode[otherNodeID];
					bool haveCommonData = otherData.sendValues.TryGetValue(thisNodeID, out T[] dataToSend);
					if (!haveCommonData)
					{
						continue;
					}

					// Just copy references to buffers. 
					thisData.recvValues[otherNodeID] = dataToSend;
					otherData.recvValues[thisNodeID] = thisData.sendValues[otherNodeID];
				}
			});
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
