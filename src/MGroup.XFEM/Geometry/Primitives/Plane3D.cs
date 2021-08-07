using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.Geometry.Primitives
{
    public class Plane3D : ISurface3D
    {
        private const int dim = 3;

        /// <summary>
        /// a * x + b * y + c * z + d = 0
        /// </summary>
        private readonly double a, b, c, d;

        /// <summary>
        /// Points towards the positive halfspace.
        /// </summary>
        private readonly Vector unitNormal;

        private readonly Vector pointOnPlane;

        public int Dimension => dim;

        private Plane3D(double a, double b, double c, double d, Vector unitNormal, Vector pointOnPlane)
        {
            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
            this.unitNormal = unitNormal;
            this.pointOnPlane = pointOnPlane;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="pointOnPlane">A point that lies on the plane.</param>
        /// <param name="positiveNormal">
        /// A normal vector that points towards the positive halfspace. It does not have to be unit.
        /// </param>
        /// <returns></returns>
        public static Plane3D CreatePlaneThroughPointWithPositiveNormal(double[] pointOnPlane, double[] positiveNormal)
        {
            var p0 = Vector.CreateFromArray(pointOnPlane);
            var unitNormal = Vector.CreateFromArray(positiveNormal);
            unitNormal.ScaleIntoThis(1 / unitNormal.Norm2());

            // Coefficients of the equation
            double a = unitNormal[0];
            double b = unitNormal[1];
            double c = unitNormal[2];
            double d = -(a * p0[0] + b * p0[1] + c * p0[2]);

            return new Plane3D(a, b, c, d, unitNormal, p0);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="pointsOnPlane">Points that lie on the plane.</param>
        /// <param name="pointOffPlane">
        /// A point that lies off the the plane, either in the positive or negative halfspace.
        /// </param>
        /// <param name="signedDistanceOfPointOffPlane">
        /// The signed distance from the plane to<paramref name="pointOffPlane"/>.
        /// </param>
        /// <returns></returns>
        public static Plane3D CreatePlaneThrough3Points(List<double[]> pointsOnPlane, double[] pointOffPlane,
            double signedDistanceOfPointOffPlane)
        {
            if (pointsOnPlane.Count != 3) throw new ArgumentException();
            if (signedDistanceOfPointOffPlane == 0)
            {
                throw new ArgumentException("The signed distance must be non-negative");
            }

            // Let P0 be a point on the plane
            Vector p0 = Vector.CreateFromArray(pointsOnPlane[0], true);

            // Form 2 vectors u,v on the plane:
            Vector u = Vector.CreateFromArray(pointsOnPlane[1]) - p0;
            Vector v = Vector.CreateFromArray(pointsOnPlane[2]) - p0;

            // Their cross product w is a vector normal to the plane.
            Vector w = u.CrossProduct(v);

            // We need to find if w points to the positive half space or not.
            // Take a vector from P0 to a point P1 off the plane and project it onto the normal w
            Vector t = Vector.CreateFromArray(pointOffPlane) - p0;
            double proj = t * w;

            // Decide which side is the correct by comparing the projection to the signed distance
            Vector unitNormal;
            if (proj * signedDistanceOfPointOffPlane > 0)
            {
                unitNormal = w.Scale(1 / w.Norm2());
            }
            else
            {
                unitNormal = w.Scale(-1 / w.Norm2());
            }

            // Coefficients of the equation
            double a = unitNormal[0];
            double b = unitNormal[1];
            double c = unitNormal[2];
            double d = -(a * p0[0] + b * p0[1] + c * p0[2]);

            return new Plane3D(a, b, c, d, unitNormal, p0);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="pointsOnPlane">Points that lie on the plane.</param>
        /// <param name="pointOffPlane">A point that lies off the the plane, either in the positive or negative halfspace.</param>
        /// <param name="signedDistanceOfPointOffPlane">The signed distance from the plane to<paramref name="pointOffPlane"/>.</param>
        /// <returns></returns>
        public static Plane3D FitPlaneThroughPoints(List<double[]> pointsOnPlane, double[] pointOffPlane,
            double signedDistanceOfPointOffPlane)
        {
            if (pointsOnPlane.Count == 3)
            {
                return CreatePlaneThrough3Points(pointsOnPlane, pointOffPlane, signedDistanceOfPointOffPlane);
            }

            //TODO: Implement this properly. For now the method just uses the first 3 points.
            //      The proper implementation needs SVD. Least squares minimizes the distance in z axis, not the perpendicular 
            //      distance.
            var threePointsOnPlane = new List<double[]>();
            threePointsOnPlane.Add(pointsOnPlane[0]);
            threePointsOnPlane.Add(pointsOnPlane[1]);
            threePointsOnPlane.Add(pointsOnPlane[2]);
            return CreatePlaneThrough3Points(threePointsOnPlane, pointOffPlane, signedDistanceOfPointOffPlane);
        }

        public double SignedDistanceOf(double[] point)
        {
            // Project the vector from P0 on the plain to the point P onto the unit normal vector, 
            // which is directed towards the positive halfspace. 
            return (Vector.CreateFromArray(point) - pointOnPlane) * unitNormal;
        }

        public double[] IntersectLineSegment(double[] point0, double[] point1)
        {
            throw new NotImplementedException();
        }
    }
}
