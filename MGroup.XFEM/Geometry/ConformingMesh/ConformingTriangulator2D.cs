using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Geometry.Tolerances;

//TODO: Allow the option to specify the minimum triangle area.
namespace MGroup.XFEM.Geometry.ConformingMesh
{
    /// <summary>
    /// Does not work correctly if an element is intersected by more than one curves, which also intersect each other.
    /// </summary>
    public class ConformingTriangulator2D : IConformingTriangulator
    {
        IElementSubcell[] IConformingTriangulator.FindConformingMesh(
            IXFiniteElement element, IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance)
            => FindConformingMesh(element, intersections, meshTolerance);

        public ElementSubtriangle2D[] FindConformingMesh(IXFiniteElement element, 
            IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance)
        {
            // Store the nodes and all intersection points in a set
            double tol = meshTolerance.CalcTolerance(element);
            var comparer = new Point2DComparer(tol);
            var nodes = new SortedSet<double[]>(comparer);
            nodes.UnionWith(element.Interpolation.NodalNaturalCoordinates);

            // Store the nodes and all intersection points in a different set
            var triangleVertices = new SortedSet<double[]>(comparer);
            triangleVertices.UnionWith(nodes);

            // Add intersection points from each curve-element intersection object.
            foreach (IElementDiscontinuityInteraction intersection in intersections)
            {
                // If the curve does not intersect this element (e.g. it conforms to the element edge), 
                // there is no need to take into account for triangulation
                if (intersection.RelativePosition != RelativePositionCurveElement.Intersecting) continue;

                IList<double[]> newVertices = intersection.GetVerticesForTriangulation();
                int countBeforeInsertion = triangleVertices.Count;
                triangleVertices.UnionWith(newVertices);

                if (triangleVertices.Count == countBeforeInsertion)
                {
                    // Corner case: the curve intersects the element at 2 opposite nodes. In this case also add the middle of their 
                    // segment to force the Delauny algorithm to conform to the segment.
                    //TODO: I should use constrained Delauny in all cases and conform to the intersection segment.
                    double[] p0 = newVertices[0];
                    double[] p1 = newVertices[1];
                    if (nodes.Contains(p0) && nodes.Contains(p1))
                    {
                        triangleVertices.Add(new double[] { 0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1]) });
                    }
                }
            }

            var triangulator = new MIConvexHullTriangulator2D();
            triangulator.MinTriangleArea = tol * element.CalcBulkSizeNatural();
            IList<Triangle2D> delaunyTriangles = triangulator.CreateMesh(triangleVertices);
            var subtriangles = new ElementSubtriangle2D[delaunyTriangles.Count];
            for (int t = 0; t < delaunyTriangles.Count; ++t)
            {
                subtriangles[t] = new ElementSubtriangle2D(delaunyTriangles[t]);
            }
            return subtriangles;
        }
    }
}
