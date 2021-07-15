using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Environments;
using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DDM
{
    /// <remarks>
    /// In the current design, the subdomain neighbors and their common (boundary) nodes are supposed to remain constant 
    /// throughout the analysis. The dofs at these nodes may be different per subdomain and even change during the analysis. 
    /// </remarks>
    public class SubdomainTopology
    {
        private readonly Dictionary<int, SortedSet<int>> neighborsPerSubdomain;

        /// <summary>
        /// The common nodes for two neighbors s0, s1 will be found and stored twice: once for s0 and once for s1.
        /// </summary>
        private readonly Dictionary<int, Dictionary<int, SortedSet<int>>> commonNodesWithNeighborsPerSubdomain;

        public SubdomainTopology(IComputeEnvironment environment, IModel model)
        {
            Func<int, SortedSet<int>> findSubdomainNeighbors = subdomainID =>
            {
                ComputeNode computeNode = environment.GetComputeNode(subdomainID);
                var neighbors = new SortedSet<int>();
                neighbors.UnionWith(computeNode.Neighbors);
                return neighbors;
            };
            this.neighborsPerSubdomain = environment.CreateDictionaryPerNode(findSubdomainNeighbors);

            Func<int, Dictionary<int, SortedSet<int>>> findCommonNodes = subdomainID =>
            {
                ISubdomain subdomain = model.GetSubdomain(subdomainID);
                var commonNodesOfThisSubdomain = new Dictionary<int, SortedSet<int>>();
                foreach (INode node in subdomain.Nodes)
                {
                    if (node.SubdomainsDictionary.Count == 1) continue; // internal node

                    foreach (int otherSubdomainID in node.SubdomainsDictionary.Keys)
                    {
                        if (otherSubdomainID == subdomainID) continue; // one of all will the current subdomain

                        Debug.Assert(neighborsPerSubdomain[subdomainID].Contains(otherSubdomainID), 
                            $"Subdomain {otherSubdomainID} is not listed as a neighbor of subdomain {subdomainID}," +
                            $" but node {node.ID} exists in both subdomains");

                        bool subdomainPairExists = commonNodesOfThisSubdomain.TryGetValue(
                            otherSubdomainID, out SortedSet<int> commonNodes);
                        if (!subdomainPairExists)
                        {
                            commonNodes = new SortedSet<int>();
                            commonNodesOfThisSubdomain[otherSubdomainID] = commonNodes;
                        }
                        commonNodes.Add(node.ID);
                    }
                }
                return commonNodesOfThisSubdomain;
            };
            this.commonNodesWithNeighborsPerSubdomain = environment.CreateDictionaryPerNode(findCommonNodes);
        }

        //TODOMPI: this is not very safe. It is easy to mix up the two subdomains, which will lead to NullReferenceException if
        //      they belong to different clusters/MPI processes. Perhaps this info should be given together with neighborsPerSubdomain
        //      to avoid such cases. Or ISubdomain could contain both of these data.

        /// <summary>
        /// Find the nodes of subdomain <paramref name="localSubdomainID"/> that are common with subdomain 
        /// <paramref name="neighborSubdomainID"/>. The order of these two subdomains is important.
        /// </summary>
        /// <param name="localSubdomainID">
        /// The main subdomain being processed by the current execution unit.
        /// </param>
        /// <param name="neighborSubdomainID">A neighboring subdomain of <paramref name="localSubdomainID"/>.</param>
        public SortedSet<int> GetCommonNodesOfSubdomains(int localSubdomainID, int neighborSubdomainID)
            => commonNodesWithNeighborsPerSubdomain[localSubdomainID][neighborSubdomainID];

        public SortedSet<int> GetNeighborsOfSubdomain(int subdomainID) => neighborsPerSubdomain[subdomainID];

        //TODOMPI: Avoid finding and storing the common nodes of a subdomain pair twice. Actually, the GetCommonNodesOfSubdomains 
        //      is safer this way.
        #region premature optimization 
        ///// <summary>
        ///// First key = subdomain with min id. Second key = subdomain with max id. Value = common nodes between the 2 subdomains.
        ///// This way ids of the common nodes are only stored once. 
        ///// </summary>
        //private readonly ConcurrentDictionary<int, ConcurrentDictionary<int, SortedSet<int>>> commonNodesWithNeighbors;

        //public SortedSet<int> GetCommonNodesOfSubdomains(int subdomain0, int subdomain1)
        //{
        //    if (subdomain0 < subdomain1)
        //    {
        //        return commonNodesWithNeighbors[subdomain0][subdomain1];
        //    }
        //    else
        //    {
        //        Debug.Assert(subdomain0 > subdomain1, "Requesting the common nodes of a subdomain with itself is illegal.");
        //        return commonNodesWithNeighbors[subdomain1][subdomain0];
        //    }
        //}
        #endregion
    }
}
