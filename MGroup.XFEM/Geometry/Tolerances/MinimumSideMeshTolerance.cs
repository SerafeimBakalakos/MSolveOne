using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Geometry.Tolerances
{
    public class MinimumSideMeshTolerance : IMeshTolerance
    {
        private readonly double coeff;

        public MinimumSideMeshTolerance(double coeff = 1E-8)
        {
            this.coeff = coeff;
        }

        public double CalcTolerance(IXFiniteElement element)
        {
            double min = double.MaxValue;
            int numNodes = element.Nodes.Count;
            for (int i = 0; i < numNodes; ++i)
            {
                XNode node1 = element.Nodes[i];
                XNode node2 = element.Nodes[(i + 1) % numNodes];
                double edgeLength = node2.CalculateDistanceFrom(node1);
                if (edgeLength < min) min = edgeLength;
            }
            return coeff * min;
        }
    }
}
