using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Environments
{
    public class ComputeNodeCluster
    {
        public ComputeNodeCluster(int id)
        {
            this.ID = id;
        }

        public int ID { get; }

        public Dictionary<int, ComputeNode> Nodes { get; } = new Dictionary<int, ComputeNode>();
    }
}
