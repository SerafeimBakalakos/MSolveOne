using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.Boundaries
{
    public interface IDomainBoundary
    {
        bool SurroundsPoint(double[] point);
    }
}
