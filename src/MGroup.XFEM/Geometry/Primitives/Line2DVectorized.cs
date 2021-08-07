using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra;
using MGroup.LinearAlgebra.Matrices;

namespace MGroup.XFEM.Geometry.Primitives
{
    /// <summary>
    /// Parametric line in 2D: R(t) = <see cref="A"/> + <see cref="s"/> * t.
    /// The line is directed: it divides the plane into a positive and a negative semiplane
    /// </summary>
    public class Line2DVectorized : ICurve2D
    {
        /// <summary>
        /// Point on the line
        /// </summary>
        private readonly double[] A;

        /// <summary>
        /// Unit vector tangent to the line and in the same direction.
        /// </summary>
        private readonly double[] s;

        /// <summary>
        /// Unit vector normal to the line in the direction obtained by a PI/2 counter-clockwise rotation to 
        /// <see cref="s"/>.
        /// </summary>
        private readonly double[] n;

        public int Dimension => 2;

        /// <summary>
        /// Directed from <paramref name="point0"/> to <paramref name="point1"/>.
        /// </summary>
        /// <param name="point0"></param>
        /// <param name="point1"></param>
        /// <param name="sys"></param>
        public Line2DVectorized(double[] point0, double[] point1)
        {
            this.A = point0;

            // Tangent unit vector
            double dx0 = point1[0] - point0[0];
            double dx1 = point1[1] - point0[1];
            double distance = Math.Sqrt(dx0 * dx0 + dx1 * dx1);
            this.s = new double[] { dx0 / distance, dx1 / distance };

            // Normal unit vector see https://matthew-brett.github.io/teaching/rotation_2d.html
            this.n = new double[] { -s[1], s[0] };
        }

        
        public double SignedDistanceOf(double[] point)
        {
            PointProjection projection = new PointProjection(A, s, point);

            // Sign
            double PPo_n = projection.PPo.DotProduct2D(n);
            if (PPo_n <= 0) return projection.Distance; // PPo, n have the same direction
            else return -projection.Distance; // PPo, n have opposite directions
        }

        /// <summary>
        /// Returns a characterization of the relative position and a list of values for the parameter t. Each value corresponds
        /// to an intersection point: r(t) = A + t * s
        /// </summary>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        /// <returns></returns>
        public IElementDiscontinuityInteraction IntersectPolygon(IList<double[]> nodes)
        {
            //TODO: Use the results to create a new geometric object that can provide vertices for triangulation, gauss points etc. These can be empty
            //TODO: needs a fast way to eleminate most elements

            // Find the projection and perpendicular vectors of these points onto the line. See SignedDistance() for more details
            var projections = new PointProjection[nodes.Count];
            for (int i = 0; i < nodes.Count; ++i)
            {
                projections[i] = new PointProjection(A, s, nodes[i]);
            }

            // Intersect each segment
            var intersections = new SortedSet<double>();
            bool conformingSegment = false;
            for (int i = 0; i < nodes.Count; ++i)
            {
                int nextIndex = (i + 1) % nodes.Count;
                (RelativePositionClosedCurves pos, double[] segmentIntersections) 
                    = IntersectSegment(projections[i], projections[nextIndex]);
                if (pos == RelativePositionClosedCurves.Conforming) conformingSegment = true;
                foreach (double t in segmentIntersections) intersections.Add(t);
            }

            // Investigate the intersection type
            if (intersections.Count == 0)
            {
                return new NullElementDiscontinuityInteraction(-1, null);
                //return (RelativePositionCurveDisc.Disjoint, new double[0]);
            }
            else if (intersections.Count == 1)
            {
                return new NullElementDiscontinuityInteraction(-1, null);
                //return (RelativePositionCurveDisc.Tangent, new double[] { intersections.First() });
            }
            else if (intersections.Count == 2)
            {
                double[] intersectionsLocal = intersections.ToArray();
                double[] start = ProjectLocalToGlobal(intersectionsLocal[0]);
                double[] end = ProjectLocalToGlobal(intersectionsLocal[1]);
                if (conformingSegment)
                {
                    return new LineSegmentIntersection2D(-1, start, end, null, RelativePositionCurveElement.Conforming);
                    //return (RelativePositionCurveDisc.Conforming, intersections.ToArray());
                }
                else
                {
                    return new LineSegmentIntersection2D(-1, start, end, null, RelativePositionCurveElement.Intersecting);
                    //return (RelativePositionCurveDisc.Intersecting, intersections.ToArray());
                }
            }
            else throw new Exception();
        }

        public double[] NormalVectorThrough(double[] point)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Returns a characterization of the relative position and a list of values for the parameter t. Each value corresponds
        /// to an intersection point: r(t) = A + t * s
        /// </summary>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        /// <returns></returns>
        private (RelativePositionClosedCurves, double[]) IntersectSegment(PointProjection proj1, PointProjection proj2)
        { //TODO: Use the results to create a new geometric object that can provide vertices for triangulation, gauss points etc. These can be empty
            
            double dot = proj1.PPo.DotProduct2D(proj2.PPo);
            if (dot > 0) // Parallel or otherwise disjoint
            {
                return (RelativePositionClosedCurves.Disjoint, new double[0]);
            }
            else if (dot < 0) // P1, P2 lie on opposite semiplanes
            {
                // Use linear interpolation
                double lambda = proj1.Distance / (proj1.Distance + proj2.Distance);
                double[] P1o = A.Subtract(proj1.PoA);
                double[] P2o = A.Subtract(proj2.PoA);
                double[] P1oP2o = P2o.Subtract(P1o);
                double[] P1oK = P1o.Add(P1oP2o.Scale(lambda));
                double tk = P1oK.DotProduct2D(s);
                return (RelativePositionClosedCurves.Intersecting, new double[] { tk });
            }
            else
            {
                double t1 = -proj1.PoA.DotProduct2D(s); // parameter for P10
                double t2 = -proj2.PoA.DotProduct2D(s); // parameter for P20

                if (proj1.Distance == 0 && proj2.Distance == 0) // Both P1 and P2 lie on the line
                {
                    return (RelativePositionClosedCurves.Conforming, Sort(t1, t2));
                }
                else if (proj1.Distance == 0 /*&& proj2.Distance != 0*/) // Only P1 lies on the line
                {
                    return (RelativePositionClosedCurves.Intersecting, new double[] { t1 });
                }
                else /*(proj1.Distance != 0 && proj2.Distance == 0)*/ // Only P2 lies on the line
                {
                    return (RelativePositionClosedCurves.Intersecting, new double[] { t2 });
                }
            }
        }

        private double[] ProjectLocalToGlobal(double t)
        {
            return A.Add(s.Scale(t));
        }

        private double[] Sort(double val1, double val2)
        {
            if (val1 < val2) return new double[] { val1, val2 };
            else if (val1 > val2) return new double[] { val2, val1 };
            else throw new NotImplementedException();
        }

        /// <summary>
        /// See https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Vector_formulation
        /// </summary>
        private class PointProjection
        {
            /// <summary>
            /// Unsigned distance
            /// </summary>
            public double Distance { get; }

            /// <summary>
            /// The segment from the investigated point to the point on this line
            /// </summary>
            public double[] PA { get; }

            /// <summary>
            /// The signed length of the projection of PA onto this line
            /// </summary>
            public double PA_s { get; }

            /// <summary>
            /// The projection vector of PA onto this line
            /// </summary>
            public double[] PoA { get; }

            /// <summary>
            /// Vector from P to Po, which is perpendicular to the line: PPo = AP - PoA
            /// </summary>
            public double[] PPo { get; }

            public PointProjection(double[] A, double[] s, double[] P)
            {
                // Po is the closest point of the line to p
                PA = A.Subtract2D(P);
                PA_s = PA.DotProduct2D(s);
                PoA = s.Scale2D(PA_s);
                PPo = PA.Subtract2D(PoA);
                Distance = PPo.Norm2D();
            }
        }
    }
}
