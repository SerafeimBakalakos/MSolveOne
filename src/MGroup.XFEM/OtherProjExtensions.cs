using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM
{
    public static class OtherProjExtensions
    {
        public static double[] Coordinates(this INode node)
        {
            if (node is XNode xnode) return xnode.Coordinates;
            else return new double[] { node.X, node.Y, node.Z };
        }
    }
}
