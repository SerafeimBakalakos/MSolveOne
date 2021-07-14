using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.Environments.Mpi;
using MGroup.Environments.Tests.Topologies;
using Xunit;

namespace MGroup.Environments.Tests.Mpi
{
    public static class MpiTagsTests
    {
        [Fact]
        public static void TestLocalNeighbors()
        {
            var topology = new Hexagon1DTopology().CreateNodeTopology();
            foreach (ComputeNodeCluster cluster in topology.Clusters.Values)
            {
                var mpiTags = new MpiP2PTransfers(topology, cluster);
                foreach (int nodeID in cluster.Nodes.Keys)
                {
                    int[] localNodesExpected = GetLocalNeighbors()[nodeID];
                    int[] localNodesComputed = mpiTags.GetLocalNeighborsOf(nodeID).ToArray();
                    Assert.True(Utilities.AreEqual(localNodesExpected, localNodesComputed));

                }
            }
        }

        [Fact]
        public static void TestRemoteNeighbors()
        {
            var topology = new Hexagon1DTopology().CreateNodeTopology();
            foreach (ComputeNodeCluster cluster in topology.Clusters.Values)
            {
                var mpiTags = new MpiP2PTransfers(topology, cluster);
                foreach (int nodeID in cluster.Nodes.Keys)
                {
                    int[] remoteNodesExpected = GetRemoteNeighbors()[nodeID];
                    int[] remoteNodesComputed = mpiTags.GetRemoteNeighborsOf(nodeID).ToArray();
                    Assert.True(Utilities.AreEqual(remoteNodesExpected, remoteNodesComputed));
                }
            }
        }

        [Fact]
        public static void TestSendRecvTags()
        {
            var topology = new Hexagon1DTopology().CreateNodeTopology();
            

            foreach (ComputeNodeCluster cluster in topology.Clusters.Values)
            {
                var mpiTags = new MpiP2PTransfers(topology, cluster);
                foreach (int nodeID in cluster.Nodes.Keys)
                {
                    foreach (MpiJob job in Enum.GetValues(typeof(MpiJob)))
                    {
                        foreach (int remoteNeighborID in GetRemoteNeighbors()[nodeID])
                        {
                            int computedTag = mpiTags.GetSendRecvTag(job, nodeID, remoteNeighborID);
                            int expectedTag = GetTag(job, nodeID, remoteNeighborID);
                            Assert.Equal(expectedTag, computedTag);
                        }
                    }
                }
            }
        }

        private static Dictionary<int, int[]> GetLocalNeighbors()
        {
            var localNodes = new Dictionary<int, int[]>();
            localNodes[0] = new int[] { 1 };
            localNodes[1] = new int[] { 0 };
            localNodes[2] = new int[] { 3 };
            localNodes[3] = new int[] { 2 };
            localNodes[4] = new int[] { 5 };
            localNodes[5] = new int[] { 4 };
            return localNodes;
        }

        private static Dictionary<int, int[]> GetRemoteNeighbors()
        {
            var remoteNodes = new Dictionary<int, int[]>();
            remoteNodes[0] = new int[] { 5 };
            remoteNodes[1] = new int[] { 2 };
            remoteNodes[2] = new int[] { 1 };
            remoteNodes[3] = new int[] { 4 };
            remoteNodes[4] = new int[] { 3 };
            remoteNodes[5] = new int[] { 0 };
            return remoteNodes;
        }

        private static int GetTag(MpiJob job, int node0, int node1)
        {
            if (job == MpiJob.TransferBufferLengthDuringNeighborhoodAllToAll)
            {
                if ((Math.Min(node0, node1) == 0) && (Math.Max(node0, node1) == 5))
                {
                    return 0;
                }
                else if ((Math.Min(node0, node1) == 1) && (Math.Max(node0, node1) == 2))
                {
                    return 0;
                }
                else if ((Math.Min(node0, node1) == 3) && (Math.Max(node0, node1) == 4))
                {
                    return 0;
                }
                else
                {
                    throw new ArgumentException($"Nodes {node0}, {node1} are not neighboring remote nodes.");
                }
            }
            else if (job == MpiJob.TransferBufferDuringNeighborhoodAllToAll)
            {
                if ((Math.Min(node0, node1) == 0) && (Math.Max(node0, node1) == 5))
                {
                    return 1;
                }
                else if ((Math.Min(node0, node1) == 1) && (Math.Max(node0, node1) == 2))
                {
                    return 1;
                }
                else if ((Math.Min(node0, node1) == 3) && (Math.Max(node0, node1) == 4))
                {
                    return 1;
                }
                else
                {
                    throw new ArgumentException($"Nodes {node0}, {node1} are not neighboring remote nodes.");
                }
            }
            else
            { 
                throw new NotImplementedException(); 
            }
        }
    }
}
