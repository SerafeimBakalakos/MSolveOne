using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.Primitives
{
    public interface IClosedManifold
    {
        int Dimension { get; }

        double SignedDistanceOf(double[] point);
    }
}
