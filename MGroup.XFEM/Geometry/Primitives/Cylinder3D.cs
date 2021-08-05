using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.Geometry.Primitives
{
    public class Cylinder3D : ISurface3D
    {
        public Cylinder3D(double[] start, double[] end, double radius)
        {
            this.Start = start;
            this.End = end;
            this.Radius = radius;

            DirectionUnit = Vector.CreateFromArray(end) - Vector.CreateFromArray(start);
            Length = DirectionUnit.Norm2();
            DirectionUnit.ScaleIntoThis(1.0 / Length);
        }

        public double[] Start { get; }

        public double[] End { get; }

        public double Radius { get; set; }
        public double Length { get; set; }

        /// <summary>
        /// Unit vector that points from <see cref="Start"/> to <see cref="End"/>.
        /// </summary>
        public Vector DirectionUnit { get; set; }

        public int Dimension => 3;

        public double SignedDistanceOf(double[] point)
        {
            // Find the projection P0 of P onto the cylider's axis AB
            var pA = Vector.CreateFromArray(Start);
            var pP = Vector.CreateFromArray(point);
            double proj = (pP - pA) * DirectionUnit;
            Vector pP0 = pA.Axpy(DirectionUnit, proj);
            double distanceFromAxis = (pP - pP0).Norm2();
            double distanceFromOutline = distanceFromAxis - Radius;

            // The cylinder is not infinite. Distances from the 2 bases must be taken into account.
            if (proj < 0)
            {
                if (distanceFromOutline < 0)
                {
                    return -proj;
                }
                else
                {
                    double distanceFromBase = -proj;
                    return Math.Sqrt(distanceFromOutline * distanceFromOutline + distanceFromBase * distanceFromBase);
                }
            }
            else if (proj > Length)
            {
                if (distanceFromOutline < 0)
                {
                    return proj - Length;
                }
                else
                {
                    double distanceFromBase = proj - Length;
                    return Math.Sqrt(distanceFromOutline * distanceFromOutline + distanceFromBase * distanceFromBase);
                }
            }
            else
            {
                return distanceFromOutline;
            }
        }
    }
}
