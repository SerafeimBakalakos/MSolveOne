using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Geometry.Tolerances
{
    public class ArbitrarySideMeshTolerance : IMeshTolerance
    {
        private readonly double coeff;

        public ArbitrarySideMeshTolerance(double coeff = 1E-8)
        {
            this.coeff = coeff;
        }

        public double CalcTolerance(IXFiniteElement element)
        {
            XNode node1 = element.Nodes[0];
            XNode node2 = element.Nodes[1];
            double edgeLength = node2.CalculateDistanceFrom(node1);
            return coeff * edgeLength;
        }
    }
}
