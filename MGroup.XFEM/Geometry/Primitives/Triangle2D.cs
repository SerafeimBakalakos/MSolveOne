using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;

namespace MGroup.XFEM.Geometry.Primitives
{
    public class Triangle2D
    {
        public Triangle2D()
        {
            Vertices = new double[3][];
        }

        public Triangle2D(double[] point0, double[] point1, double[] point2)
        {
            Vertices = new double[3][] { point0, point1, point2 };
        }

        public IList<double[]> Vertices { get; }

        public double CalcArea() => Utilities.CalcPolygonArea(Vertices);
    }
}
