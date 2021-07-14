using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

//TODOMPI: This needs testing itself. Way to easy to make mistakes here. Serial code is sufficient. 
namespace MGroup.Environments.Mpi
{
    public class MpiP2PTransfers
    {
        private readonly ComputeNodeTopology nodeTopology;
        private readonly Dictionary<int, SortedSet<int>> localNeighbors; // The order is important
        private readonly Dictionary<int, SortedSet<int>> remoteNeighbors; // The order is important
        private readonly Dictionary<MpiJob, Dictionary<int, Dictionary<int, int>>> sendRecvTags;

        public MpiP2PTransfers(ComputeNodeTopology nodeTopology, ComputeNodeCluster localCluster)
        {
            this.nodeTopology = nodeTopology;

            localNeighbors = new Dictionary<int, SortedSet<int>>();
            remoteNeighbors = new Dictionary<int, SortedSet<int>>();

            // Keys: remote clusters with 1 or more nodes that are neighbors with the nodes of the local cluster.
            // Values: the unique pairs of neighboring nodes, one which belongs to the local cluster and the other at the 
            // corresponding remote.
            var allUniquePairs = new Dictionary<int, UniquePairs>();

            // Find the local and remote neighbors of nodes in the local cluster, as well as neighboring remote clusters.
            foreach (ComputeNode node in localCluster.Nodes.Values)
            {
                var localNeighborsOfNode = new SortedSet<int>();
                var remoteNeighborsOfNode = new SortedSet<int>();
                foreach (int neighborNodeID in node.Neighbors)
                {
                    if (node.Cluster.Nodes.ContainsKey(neighborNodeID))
                    {
                        localNeighborsOfNode.Add(neighborNodeID);
                    }
                    else
                    {
                        remoteNeighborsOfNode.Add(neighborNodeID);

                        int neighborClusterID = nodeTopology.Nodes[neighborNodeID].Cluster.ID;
                        bool isClusterProcessed = allUniquePairs.TryGetValue(neighborClusterID, out UniquePairs pairs);
                        if (!isClusterProcessed)
                        {
                            pairs = new UniquePairs();
                            allUniquePairs[neighborClusterID] = pairs;
                        }
                        pairs.AddPair(node.ID, neighborNodeID);
                    }
                }
                localNeighbors[node.ID] = localNeighborsOfNode;
                remoteNeighbors[node.ID] = remoteNeighborsOfNode;
            }

            // Give a tag for MPI send/receive calls, to each local-remote pair of neighboring nodes.
            sendRecvTags = new Dictionary<MpiJob, Dictionary<int, Dictionary<int, int>>>();
            foreach (MpiJob job in Enum.GetValues(typeof(MpiJob)))
            {
                sendRecvTags[job] = new Dictionary<int, Dictionary<int, int>>();
            }
            foreach (int neighborClusterID in allUniquePairs.Keys)
            {
                UniquePairs commonPairs = allUniquePairs[neighborClusterID];
                int tag = 0;
                foreach (MpiJob job in Enum.GetValues(typeof(MpiJob)))
                {
                    //TODO: It will be faster to receive a SortedSet<int> pair minNodeID: fewer tuples and lookups at localCluster.Nodes
                    foreach ((int minNodeID, int maxNodeID) in commonPairs.EnumeratePairs())
                    {
                        int localNodeID, remoteNodeID;
                        if (localCluster.Nodes.ContainsKey(minNodeID))
                        {
                            Debug.Assert(!localCluster.Nodes.ContainsKey(maxNodeID));
                            localNodeID = minNodeID;
                            remoteNodeID = maxNodeID;
                        }
                        else
                        {
                            Debug.Assert(localCluster.Nodes.ContainsKey(maxNodeID));
                            localNodeID = maxNodeID;
                            remoteNodeID = minNodeID;
                        }

                        if (!sendRecvTags[job].ContainsKey(localNodeID))
                        {
                            sendRecvTags[job][localNodeID] = new Dictionary<int, int>();
                        }
                        sendRecvTags[job][localNodeID][remoteNodeID] = tag++;
                    }
                }
            }
        }

        public IEnumerable<int> GetLocalNeighborsOf(int nodeID) => localNeighbors[nodeID];

        public IEnumerable<int> GetRemoteNeighborsOf(int nodeID) => remoteNeighbors[nodeID];

        /// <summary>
        /// Gets a tag for MPI send (in source process) and corresponding receive (in destination process) operation. This tag 
        /// is unique for transfering data from the <see cref="ComputeNode"/> with id=<paramref name="localNodeID"/> to 
        /// the <see cref="ComputeNode"/> with id=<paramref name="remoteNodeID"/>. In addition, tags for different 
        /// <see cref="MpiJob"/>s will be different, even if the source and destination nodes are the same. If tag0 refers to
        /// send/receive operation between <see cref="ComputeNode"/>s n0, n1 which belong to <see cref="ComputeNodeCluster"/>
        /// c0 and tag1 refers send/receive operation between n2, n3 which belong to c1, then these 2 tags may be the same. This 
        /// is not a problem, since the MPI send/receive calls will differentiate between the processes p0 and p1 that correspond
        /// to c0, c1 respectively. Furthermore this overlap does not necessitate a global numbering of tags across all nodes 
        /// and clusters; only the transfers between 2 neighboring clusters are needed for numbering the tags, which is far more 
        /// scalable. The tag for send/receive from <see cref="ComputeNode"/> n0 to <see cref="ComputeNode"/> n1 is the same
        /// as the tag for send/receive from n1 to n0, since they cannot be mixed up.
        /// </summary>
        /// <param name="job">
        /// The <see cref="MpiJob"/> for which this send/recv tag will be used. This is uniquely defined by each method of 
        /// <see cref="MpiEnvironment"/>.
        /// </param>
        /// <param name="localNodeID">
        /// The id of the <see cref="ComputeNode"/> that will send/receive data using the returned tag and belongs to the local
        /// <see cref="ComputeNodeCluster"/> managed by the MPI process that calls this method.
        /// </param>
        /// <param name="remoteNodeID">
        /// The id of the <see cref="ComputeNode"/> that will send/receive data using the returned tag and belongs to a remote
        /// <see cref="ComputeNodeCluster"/> managed by a different MPI process than the one that calls this method.
        /// </param>
        public int GetSendRecvTag(MpiJob job, int localNodeID, int remoteNodeID)
            => sendRecvTags[job][localNodeID][remoteNodeID];

        private class UniquePairs
        {
            private readonly SortedDictionary<int, SortedSet<int>> allPairs = 
                new SortedDictionary<int, SortedSet<int>>();

            public void AddPair(int item0, int item1)
            {
                Debug.Assert(item0 != item1);
                int min, max;
                if (item0 < item1)
                {
                    min = item0;
                    max = item1;
                }
                else
                {
                    min = item1;
                    max = item0;
                }

                bool minExists = allPairs.TryGetValue(min, out SortedSet<int> pairsWithMin);
                if (!minExists)
                {
                    pairsWithMin = new SortedSet<int>();
                    allPairs[min] = pairsWithMin;
                }
                pairsWithMin.Add(max);
            }

            /// <summary>
            /// The first item of each pair will be the smaller of the two, followed by the larger. The order of pairs is as 
            /// follows: first all pairs whose first item is the minimum number of all entries, followed by the pairs whose 
            /// first item is the next smallest, etc. Pairs whose first item is the same number are thus contiguous and in 
            /// ascending order of their second item. 
            /// </summary>
            public IEnumerable<(int itemMin, int itemMax)> EnumeratePairs() 
            {
                foreach (var pair in allPairs)
                {
                    int itemMin = pair.Key;
                    foreach (int itemMax in pair.Value)
                    {
                        yield return (itemMin, itemMax);
                    }
                }
            }
        }
    }
}
