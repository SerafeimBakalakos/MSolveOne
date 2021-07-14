using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Environments.Tests.Topologies
{
    public class Hexagon1DTopology
    {
        public ComputeNodeTopology CreateNodeTopology()
        {
            var topology = new ComputeNodeTopology();
            topology.AddNode(0, new int[] { 5, 1 }, 0);
            topology.AddNode(1, new int[] { 0, 2 }, 0);
            topology.AddNode(2, new int[] { 1, 3 }, 1);
            topology.AddNode(3, new int[] { 2, 4 }, 1);
            topology.AddNode(4, new int[] { 3, 5 }, 2);
            topology.AddNode(5, new int[] { 4, 0 }, 2);
            return topology;
        }
    }
}
