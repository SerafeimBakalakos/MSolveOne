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
        private ComputeNodeTopology nodeTopology;

        public TplSharedEnvironment()
        {
        }

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

        public Dictionary<int, T> CreateDictionaryPerNode<T>(Func<int, T> createDataPerNode)
        {
            // Add the keys first to avoid race conditions
            var result = new Dictionary<int, T>();
            foreach (int nodeID in nodeTopology.Nodes.Keys) 
            {
                result[nodeID] = default; 
            }

            // Run the operation per node in parallel and store the individual results.
            Parallel.ForEach(nodeTopology.Nodes.Keys, nodeID => result[nodeID] = createDataPerNode(nodeID));

            return result;
        }

        public void DoPerNode(Action<int> actionPerNode)
        {
            Parallel.ForEach(nodeTopology.Nodes.Keys, actionPerNode);
        }

        public ComputeNode GetComputeNode(int nodeID) => nodeTopology.Nodes[nodeID];

        public void Initialize(ComputeNodeTopology nodeTopology)
        {
            this.nodeTopology = nodeTopology;
        }

        public void NeighborhoodAllToAll<T>(Dictionary<int, AllToAllNodeData<T>> dataPerNode, bool areRecvBuffersKnown)
        {
            Parallel.ForEach(nodeTopology.Nodes.Keys, thisNodeID =>
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
            });
        }
    }
}
