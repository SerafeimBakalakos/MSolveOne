using System;
using System.Collections.Generic;
using System.Linq;
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
		#region debug reortho - remake them regular dictionaries instead of sorted.
		public SortedDictionary<int, ComputeNode> Nodes { get; } = new SortedDictionary<int, ComputeNode>();
		#endregion
		//public Dictionary<int, ComputeNode> Nodes { get; } = new Dictionary<int, ComputeNode>();

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

		public void CheckSanity()
		{
			int[] clusterIDs = Clusters.Keys.ToArray();
			Array.Sort(clusterIDs);
			int nextExpectedID = 0;
			for (int c = 0; c < clusterIDs.Length; ++c)
			{
				if (clusterIDs[c] == nextExpectedID)
				{
					++nextExpectedID;
				}
				else
				{
					throw new Exception("Cluster ids must be consecutive integers starting from 0.");
				}
			}

			int[] nodeIDs = Nodes.Keys.ToArray();
			Array.Sort(nodeIDs);
			nextExpectedID = 0;
			for (int n = 0; n < nodeIDs.Length; ++n)
			{
				if (nodeIDs[n] == nextExpectedID)
				{
					++nextExpectedID;
				}
				else
				{
					throw new Exception("Node ids must be consecutive integers starting from 0.");
				}
			}

			foreach (ComputeNodeCluster cluster in Clusters.Values)
			{
				if (cluster.Nodes.Count < 1)
				{
					throw new ArgumentException(
						$"Each cluster most contain at least 1 compute node, but cluster {cluster.ID} contains none.");
				}
			}
		}
	}
}
