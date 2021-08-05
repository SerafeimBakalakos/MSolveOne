using System;
using System.Collections.Generic;
//using ISAAR.MSolve.Geometry.Commons;
//using ISAAR.MSolve.Geometry.Coordinates;
//using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.Geometry.Primitives
{
    public class PolyLine2D : ICurve2D
    {
        private readonly List<double> anglesBetweenSegments;
        private readonly List<double> anglesOfSegments;
        private readonly List<LineSegment2D> segments;
        private readonly List<double[]> vertices;

        public PolyLine2D(double[] first, double[] second)
        {
            vertices = new List<double[]>();
            segments = new List<LineSegment2D>();
            anglesBetweenSegments = new List<double>();
            anglesOfSegments = new List<double>();

            vertices.Add(first);
            vertices.Add(second);
            segments.Add(LineSegment2D.Create(first, second));

            double dx = second[0] - first[0];
            double dy = second[1] - first[1];
            anglesOfSegments.Add(Math.Atan2(dy, dx));
        }

        public double[] End => vertices[vertices.Count - 1];
        public IReadOnlyList<LineSegment2D> Segments => segments;
        public double[] Start => vertices[0];
        public IReadOnlyList<double[]> Vertices => vertices;

        public int Dimension => 2;

        /// <summary>
        /// Counter-clockwise angle from global cartesian x axis to a vector which 1) starts at the end point of the 
        /// curve, 2) is tangent to the curve and 3) heads outwards from the curve.
        /// </summary>
        public double EndPointOrientation() // TODO: perhaps it should return a local coordinate system. I do not need the angle, but its cos and sign.
        {
            double[] lastSegmentStart = vertices[vertices.Count - 2];
            double[] lastSegmentEnd = vertices[vertices.Count - 1];
            double dx = lastSegmentEnd[0] - lastSegmentStart[0];
            double dy = lastSegmentEnd[1] - lastSegmentStart[1];
            return Math.Atan2(dy, dx);
        }

        public IElementDiscontinuityInteraction IntersectPolygon(IList<double[]> nodes)
        {
            throw new NotImplementedException();
        }

        // The normal vector for the positive region.
        public double[] NormalVectorThrough(double[] point)
        {
            if (segments.Count == 1)
            {
                return segments[0].NormalVectorThrough(point);
            }
            else throw new NotImplementedException();
        }

        /// <summary>
        /// See Fries's slides
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public double SignedDistanceOf(double[] point)
        {
            if (segments.Count == 1) return segments[0].TransformGlobalToLocalPoint(point)[1];

            var distances = new List<double>();
            bool afterPreviousSegment = false;

            // First segment
            double[] localPoint = segments[0].TransformGlobalToLocalPoint(point);
            if (localPoint[0] < segments[0].Length) distances.Add(localPoint[1]);
            else afterPreviousSegment = true;

            // Subsequent segments
            for (int i = 1; i < segments.Count - 1; ++i)
            {
                localPoint = segments[i].TransformGlobalToLocalPoint(point);
                if (localPoint[0] < 0.0)
                {
                    if (afterPreviousSegment)
                    {
                        // Compute the distance from the vertex between this segment and the previous
                        double dx = point[0] - vertices[i][0];
                        double dy = point[1] - vertices[i][1];
                        double distance = Math.Sqrt(dx * dx + dy * dy);
                        int sign = -Math.Sign(anglesBetweenSegments[i - 1]); // If growth angle > 0, the convex angle faces the positive area.
                        distances.Add(sign * distance);
                    }
                    afterPreviousSegment = false;
                }
                else if (localPoint[0] <= segments[i].Length)
                {
                    distances.Add(localPoint[1]);
                    afterPreviousSegment = false;
                }
                else afterPreviousSegment = true;
            }

            // Last segment
            int last = segments.Count - 1;
            localPoint = segments[last].TransformGlobalToLocalPoint(point);
            if (localPoint[0] < 0.0)
            {
                if (afterPreviousSegment)
                {
                    // Compute the distance from the vertex between this segment and the previous
                    double dx = point[0] - vertices[last][0];
                    double dy = point[1] - vertices[last][1];
                    double distance = Math.Sqrt(dx * dx + dy * dy);
                    int sign = -Math.Sign(anglesBetweenSegments[last - 1]); // If growth angle > 0, the convex angle faces the positive area.
                    distances.Add(sign * distance);
                }
                afterPreviousSegment = false;
            }
            else distances.Add(localPoint[1]);

            return distances[Utilities.IndexOfMinAbs(distances)];
        }

        /// <summary>
        /// Counter-clockwise angle from global cartesian x axis to a vector which 1) starts at the start point of the 
        /// curve, 2) is tangent to the curve and 3) heads outwards from the curve.
        /// </summary>
        public double StartPointOrientation()
        {
            // This one's orientation requires more thought, especially since the convention for determining the
            // level set value gives the opposite sign from the rest of the curve.
            double[] firstSegmentStart = vertices[0];
            double[] firstSegmentEnd = vertices[1];
            double dx = firstSegmentStart[0] - firstSegmentEnd[0];
            double dy = firstSegmentStart[1] - firstSegmentEnd[1];
            return Math.Atan2(dy, dx);
        }

        public void UpdateGeometry(double angleToLastSegment, double length)
        {
            double lastGlobalAngle = anglesOfSegments[anglesOfSegments.Count - 1];
            double newGlobalAngle = Utilities.WrapAngle(angleToLastSegment + lastGlobalAngle);
            double dx = length * Math.Cos(newGlobalAngle);
            double dy = length * Math.Sin(newGlobalAngle);

            var lastPoint = Vertices[Vertices.Count - 1];
            var newPoint = new double[] { lastPoint[0] + dx, lastPoint[1] + dy };
            vertices.Add(newPoint);
            segments.Add(LineSegment2D.Create(lastPoint, newPoint));
            anglesBetweenSegments.Add(angleToLastSegment); // These are independent of the global coordinate system
            anglesOfSegments.Add(newGlobalAngle);
        }
    }
}
