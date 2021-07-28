using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Environments
{
    public class ComputeNode
    {
        public ComputeNode(int id)
        {
            ID = id;
        }

        public ComputeNodeCluster Cluster { get; set; }

        public int ID { get; }

        public SortedSet<int> Neighbors { get; } = new SortedSet<int>();
    }
}
