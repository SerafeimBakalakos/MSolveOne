using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;

namespace MGroup.XFEM.Interpolation.Inverse
{
    /// <summary>
    /// Inverse mapping of the isoparametric interpolation of a triangular finite element with 3 nodes. Since the original 
    /// mapping is linear, there are analytic formulas. WARNING: this assumes 
    /// ShapeFunctions(xi, eta) => new double[]{ xi, eta, 1-xi-eta };
    /// </summary>
    public class InverseInterpolationTri3 : IInverseInterpolation
    {
        private readonly double x1, x2, x3, y1, y2, y3;
        private readonly double det;

        public InverseInterpolationTri3(IReadOnlyList<INode> nodes)
        {
            throw new NotImplementedException(
                "This class needs to be updated. It worked for the following order of Tri3 nodes: (1,0) (0,1) (0,0), which has now changed to (0,0) (1,0) (0,1)");
            x1 = nodes[0].X;
            x2 = nodes[1].X;
            x3 = nodes[2].X;
            y1 = nodes[0].Y;
            y2 = nodes[1].Y;
            y3 = nodes[2].Y;
            det = (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
        }

        public double[] TransformPointCartesianToNatural(double[] point)
        {
            double detXi = (point[0] - x3) * (y2 - y3) - (x2 - x3) * (point[1] - y3);
            double detEta = (x1 - x3) * (point[1] - y3) - (point[0] - x3) * (y1 - y3);
            return new double[] { detXi / det, detEta / det };
        }
    }
}
