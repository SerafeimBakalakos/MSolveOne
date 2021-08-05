using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Elements
{
    public static class XElementExtensions
    {
        private static void PreparePoint(IXFiniteElement element, XPoint point)
        {
            if (point.Element == null) point.Element = element;
            else if (point.Element != element) throw new ArgumentException("The provided point does not belong to this element");

            if (point.ShapeFunctions == null)
            {
                bool hasNatural = point.Coordinates.TryGetValue(CoordinateSystem.ElementNatural, out double[] natural);
                if (!hasNatural)
                {
                    throw new ArgumentException("Either the natural coordinates of the point or"
                        + " the shape functions of the element evaluated at it must be provided.");
                }
                point.ShapeFunctions = element.Interpolation.EvaluateFunctionsAt(natural);
            }
        }
    }
}
