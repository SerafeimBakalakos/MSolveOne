using System;
using System.Collections.Generic;

namespace MGroup.XFEM.Geometry.Primitives
{
    public enum CirclePolygonPosition
    {
        Disjoint, Intersecting, PolygonInsideCircle, CircleInsidePolygon
    }

    public enum PolygonPointPosition
    {
        Inside, Outside, OnEdge, OnVertex
    }

    public class ConvexPolygon2D
    {
        public ConvexPolygon2D(IReadOnlyList<double[]> vertices)
        {
            this.Vertices = vertices;

            var edges = new LineSegment2D[vertices.Count];
            for (int i = 0; i < vertices.Count; ++i)
            {
                edges[i] = LineSegment2D.Create(vertices[i], vertices[(i + 1) % vertices.Count]);
            }
            this.Edges = edges;
        }

        public IReadOnlyList<double[]> Vertices { get; }
        public IReadOnlyList<LineSegment2D> Edges { get; }

        public double ComputeArea()
        {
            double sum = 0.0;
            for (int vertexIdx = 0; vertexIdx < Vertices.Count; ++vertexIdx)
            {
                double[] vertex1 = Vertices[vertexIdx];
                double[] vertex2 = Vertices[(vertexIdx + 1) % Vertices.Count];
                sum += vertex1[0] * vertex2[1] - vertex2[0] * vertex1[1];
            }
            return Math.Abs(0.5 * sum); // area would be negative if vertices were in counter-clockwise order
        }

        /// <summary>
        /// Ray casting method, shamelessly copied from 
        /// http://stackoverflow.com/questions/8721406/how-to-determine-if-a-point-is-inside-a-2d-convex-polygon or 
        /// http://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon?noredirect=1&lq=1
        /// For points on the outline of the polygon, false is returned. NOPE, it isn't always.
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public bool IsPointInsidePolygon(double[] point)
        {
            int i, j;
            bool result = false;
            for (i = 0, j = Vertices.Count - 1; i < Vertices.Count; j = i++)
            {
                if (    ( (Vertices[i][1] > point[1]) != (Vertices[j][1] > point[1]) ) &&
                        ( point[0] < (Vertices[j][0] - Vertices[i][0]) * (point[1] - Vertices[i][1]) 
                                    / (Vertices[j][1] - Vertices[i][1]) + Vertices[i][0] ) )
                {
                    result = !result;
                }
            }
            return result;
        }

        public CirclePolygonPosition FindRelativePositionOfCircle(Circle2D circle)
        {
            int verticesOutsideCircle = 0;
            int verticesInsideCircle = 0;
            foreach (double[] vertex in Vertices)
            {
                double signedDistance = circle.SignedDistanceOf(vertex);
                if (signedDistance > 0) ++verticesOutsideCircle;
                else if (signedDistance < 0) ++verticesInsideCircle;
            }
            int verticesOnCircle = Vertices.Count - verticesOutsideCircle - verticesInsideCircle;

            if (verticesOutsideCircle == Vertices.Count)
            {
                if (IsPointInsidePolygon(circle.Center)) return CirclePolygonPosition.CircleInsidePolygon;
                else return CirclePolygonPosition.Disjoint;
            }
            else if (verticesOutsideCircle == 0) return CirclePolygonPosition.PolygonInsideCircle;
            else if ((verticesOnCircle == 1) && (verticesInsideCircle == 0)) return CirclePolygonPosition.Disjoint;
            else return CirclePolygonPosition.Intersecting;
        }

        public PolygonPointPosition FindRelativePositionOfPoint(double[] point, double tolerance)
        {
            int edgesPassingThroughPoint = 0;
            for (int i = 0; i < Vertices.Count; ++i)
            {
                var edge = LineSegment2D.Create(Vertices[i], Vertices[(i + 1) % Vertices.Count]);
                double signedDistance = edge.SignedDistanceOf(point);
                if (Math.Abs(signedDistance) < tolerance)
                {
                    ++edgesPassingThroughPoint;
                }
            }
            if (edgesPassingThroughPoint == 1) return PolygonPointPosition.OnEdge;
            else if (edgesPassingThroughPoint == 2) return PolygonPointPosition.OnVertex;
            else if (IsPointInsidePolygon(point)) return PolygonPointPosition.Inside;
            else 
            if (IsPointInsidePolygon(point)) return PolygonPointPosition.Inside;
            else return PolygonPointPosition.Outside;
        }
    }
}
