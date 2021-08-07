using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.Primitives
{
    public interface ICurve2D : IClosedManifold
    { 
        IElementDiscontinuityInteraction IntersectPolygon(IList<double[]> nodes);
    }
}
