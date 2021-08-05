using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;

namespace MGroup.XFEM.Geometry.Primitives
{
    public class Tetrahedron3D
    {
        public Tetrahedron3D()
        {
            Vertices = new double[4][];
        }

        public Tetrahedron3D(double[] point0, double[] point1, double[] point2, double[] point3)
        {
            Vertices = new double[4][] { point0, point1, point2, point3 };
        }

        public IList<double[]> Vertices { get; }

        internal double CalcVolume() => Utilities.CalcTetrahedronVolume(Vertices);
    }
}
