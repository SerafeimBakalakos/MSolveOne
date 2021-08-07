using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.DataStructures;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Tests.Geometry.Triangulation
{
    internal static class TriangulationUtilities
    {
        internal static bool AreEqual(double[] expected, double[] computed, double tol)
        {
            if (expected.Length != computed.Length) return false;
            var comparer = new ValueComparer(tol);
            for (int i = 0; i < expected.Length; ++i)
            {
                if (!comparer.AreEqual(expected[i], computed[i])) return false;
            }
            return true;
        }

        internal static bool AreEqual(Triangle2D expected, Triangle2D computed, double tol)
        {
            if (expected.Vertices.Count != computed.Vertices.Count) return false;
            foreach (double[] computedVertex in computed.Vertices)
            {
                bool isInExpected = false;
                foreach (double[] expectedVertex in expected.Vertices)
                {
                    if (AreEqual(expectedVertex, computedVertex, tol))
                    {
                        isInExpected = true;
                        break;
                    }
                }
                if (!isInExpected) return false;
            }
            return true;
        }

        internal static bool AreEqual(Tetrahedron3D expected, Tetrahedron3D computed, double tol)
        {
            if (expected.Vertices.Count != computed.Vertices.Count) return false;
            foreach (double[] computedVertex in computed.Vertices)
            {
                bool isInExpected = false;
                foreach (double[] expectedVertex in expected.Vertices)
                {
                    if (AreEqual(expectedVertex, computedVertex, tol))
                    {
                        isInExpected = true;
                        break;
                    }
                }
                if (!isInExpected) return false;
            }
            return true;
        }

        internal static bool AreEqual(IList<Triangle2D> expected, IList<Triangle2D> computed, double tol)
        {
            if (expected.Count != computed.Count) return false;
            foreach (Triangle2D computedTriangle in computed)
            {
                bool isInExpected = false;
                foreach (Triangle2D expectedTriangle in expected)
                {
                    if (AreEqual(expectedTriangle, computedTriangle, tol))
                    {
                        isInExpected = true;
                        break;
                    }
                }
                if (!isInExpected) return false;
            }
            return true;
        }

        internal static bool AreEqual(IList<Tetrahedron3D> expected, IList<Tetrahedron3D> computed, double tol)
        {
            if (expected.Count != computed.Count) return false;
            foreach (Tetrahedron3D computedTetra in computed)
            {
                bool isInExpected = false;
                foreach (Tetrahedron3D expectedTetra in expected)
                {
                    if (AreEqual(expectedTetra, computedTetra, tol))
                    {
                        isInExpected = true;
                        break;
                    }
                }
                if (!isInExpected) return false;
            }
            return true;
        }

        internal static double CalcPolygonArea(List<double[]> points)
        {
            double sum = 0.0;
            for (int vertexIdx = 0; vertexIdx < points.Count; ++vertexIdx)
            {
                double[] vertex1 = points[vertexIdx];
                double[] vertex2 = points[(vertexIdx + 1) % points.Count];
                sum += vertex1[0] * vertex2[1] - vertex2[0] * vertex1[1];
            }
            return Math.Abs(0.5 * sum); // area would be negative if vertices were in counter-clockwise order
        }
    }
}
