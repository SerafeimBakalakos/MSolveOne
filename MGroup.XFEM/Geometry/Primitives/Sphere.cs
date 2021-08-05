using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.Primitives
{
    public class Sphere : ISurface3D
    {
        public Sphere(double centerX, double centerY, double centerZ, double radius)
        {
            this.Center = new double[] { centerX, centerY, centerZ };
            this.Radius = radius;
        }

        public Sphere(double[] center, double radius)
        {
            this.Center = center;
            this.Radius = radius;
        }

        public double[] Center { get; }
        public double Radius { get; }

        public int Dimension => 3;

        public double SignedDistanceOf(double[] point) => Utilities.Distance3D(Center, point) - Radius;

        public double Volume() => 4.0 * Math.PI * Radius * Radius * Radius / 3.0;

        public static double CalcRadiusFromVolume(double volume) => Math.Pow(3.0 / 4.0 * volume / Math.PI, 1.0 / 3.0);
    }
}
