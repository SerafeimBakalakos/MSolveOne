using System;
using System.Collections.Generic;
using System.Text;

//TODOMPI: Perhaps this should be exposed for IComputeEnvironment
namespace MGroup.Environments
{
    /// <summary>
    /// Represents the topology of <see cref="ComputeNode"/>s used for an application. This includes neighborhoods between 
    /// <see cref="ComputeNode"/>s and clustering. All instances of this class must represent the full topology, even if
    /// some <see cref="ComputeNode"/>s will be process by resources that do not share the same memory address space.
    /// Not thread safe.
    /// </summary>
    public class ComputeNodeTopology
    {
        public Dictionary<int, ComputeNode> Nodes { get; } = new Dictionary<int, ComputeNode>();

        public Dictionary<int, ComputeNodeCluster> Clusters { get; } = new Dictionary<int, ComputeNodeCluster>();

        public void AddNode(int nodeID, IEnumerable<int> neighborNodeIDs, int clusterIDOfNode)
        {
            if (Nodes.ContainsKey(nodeID))
            {
                throw new ArgumentException($"A compute node with ID = {nodeID} already exists.");
            }

            var node = new ComputeNode(nodeID);
            Nodes[nodeID] = node;
            node.Neighbors.UnionWith(neighborNodeIDs);

            bool clusterExists = Clusters.TryGetValue(clusterIDOfNode, out ComputeNodeCluster cluster);
            if (!clusterExists)
            {
                cluster = new ComputeNodeCluster(clusterIDOfNode);
                Clusters[clusterIDOfNode] = cluster;
            }
            cluster.Nodes[nodeID] = node;
            node.Cluster = cluster;
        }
    }
}
