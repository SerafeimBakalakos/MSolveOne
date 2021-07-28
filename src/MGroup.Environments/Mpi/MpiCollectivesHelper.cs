using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;

namespace MGroup.Environments.Mpi
{
	public class MpiCollectivesHelper
	{
		private readonly int numNodes;

		private readonly Dictionary<int, int[]> localArrayIndicesToNodeIDs;

		/// <summary>
		/// In a total array (e.g after gather, before scatter/broadcast) each entry corresponds to a 
		/// <see cref="ComputeNode"/>, but they are not placed in order. In fact the nodes of a <see cref="ComputeNodeCluster"/>
		/// are placed contiguously and in order, but these nodes do not have consecutive IDs in general. This property maps
		/// the indices of node data in a aggregate array to the IDs of the corresponding nodes.
		/// </summary>
		private readonly int[] totalArrayIndicesToNodeIDs;

		public MpiCollectivesHelper(ComputeNodeTopology nodeTopology)
		{
			this.numNodes = nodeTopology.Nodes.Count;
			this.NodeCounts = GetNodeCounts(nodeTopology);
			this.totalArrayIndicesToNodeIDs = GetTotalArrayIndicesToNodeIds(nodeTopology);
			Debug.Assert(totalArrayIndicesToNodeIDs.Length == numNodes);
			this.localArrayIndicesToNodeIDs = GetLocalArrayIndicesToNodeIds(nodeTopology);
		}

		public int[] NodeCounts { get; }

		public T[] AllNodesDataToArray<T>(Dictionary<int, T> allNodesData)
		{
			var array = new T[numNodes];
			for (int n = 0; n < numNodes; ++n)
			{
				array[n] = allNodesData[totalArrayIndicesToNodeIDs[n]];
			}
			return array;
		}

		public Dictionary<int, T> AllNodesDataToDictionary<T>(T[] allNodesData)
		{
			var dictionary = new Dictionary<int, T>(numNodes);
			Debug.Assert(allNodesData.Length == numNodes);
			for (int n = 0; n < numNodes; ++n)
			{
				dictionary[totalArrayIndicesToNodeIDs[n]] = allNodesData[n];
			}
			return dictionary;
		}

		public T[] LocalNodesDataToArray<T>(int clusterID, Func<int, T> getNodeData)
		{
			int[] localNodeIDs = localArrayIndicesToNodeIDs[clusterID];
			var array = new T[localNodeIDs.Length];
			for (int n = 0; n < localNodeIDs.Length; ++n)
			{
				array[n] = getNodeData(localNodeIDs[n]);
			}
			return array;
		}

		public Dictionary<int, T> LocalNodesDataToDictionary<T>(int clusterID, T[] localNodesData)
		{
			int[] localNodeIDs = localArrayIndicesToNodeIDs[clusterID];
			var dictionary = new Dictionary<int, T>(localNodeIDs.Length);
			Debug.Assert(localNodesData.Length == localNodeIDs.Length);
			for (int n = 0; n < localNodeIDs.Length; ++n)
			{
				dictionary[localNodeIDs[n]] = localNodesData[n];
			}
			return dictionary;
		}

		private static Dictionary<int, int[]> GetLocalArrayIndicesToNodeIds(ComputeNodeTopology nodeTopology)
		{
			//TODO Optim: perhaps do this only for the local cluster at each process, instead of all clusters at each process 
			//TODO: duplication with the total array version of the method
			var result = new Dictionary<int, int[]>(nodeTopology.Clusters.Count);
			for (int c = 0; c < nodeTopology.Clusters.Count; ++c)
			{
				int[] clusterNodeIDs = nodeTopology.Clusters[c].Nodes.Keys.ToArray();
				Array.Sort(clusterNodeIDs); // Use the same order in all machines, instead of depending on the order of Dictionary.
				result[c] = clusterNodeIDs;
			}
			return result;
		}

		private static int[] GetNodeCounts(ComputeNodeTopology nodeTopology)
		{
			var counts = new int[nodeTopology.Clusters.Count];
			for (int c = 0; c < counts.Length; ++c)
			{
				counts[c] = nodeTopology.Clusters[c].Nodes.Count;
			}
			return counts;
		}

		private static int[] GetTotalArrayIndicesToNodeIds(ComputeNodeTopology nodeTopology)
		{
			var result = new int[nodeTopology.Nodes.Count];
			int clusterOffset = 0;
			for (int c = 0; c < nodeTopology.Clusters.Count; ++c)
			{
				int[] clusterNodeIDs = nodeTopology.Clusters[c].Nodes.Keys.ToArray();
				Array.Sort(clusterNodeIDs); // Use the same order in all machines, instead of depending on the order of Dictionary.
				Array.Copy(clusterNodeIDs, 0, result, clusterOffset, clusterNodeIDs.Length);
				clusterOffset += clusterNodeIDs.Length;
			}
			return result;
		}
	}
}
