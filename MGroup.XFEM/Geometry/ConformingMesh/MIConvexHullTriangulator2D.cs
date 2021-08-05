using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Geometry.Primitives;
using MIConvexHull;

namespace MGroup.XFEM.Geometry.ConformingMesh
{
    public class MIConvexHullTriangulator2D : ITriangulator2D
    {
        //TODO: Add precision controls as properties
        public MIConvexHullTriangulator2D()
        {
        }

        public double MinTriangleArea { get; set; } = -1;

        public IList<Triangle2D> CreateMesh(IEnumerable<double[]> points)
        {
            // Gather the vertices
            List<double[]> vertices = points.ToList();

            // Call 3rd-party mesh generator
            var meshCells = Triangulation.CreateDelaunay(vertices).Cells.ToArray();

            // Repackage the triangle cells
            var triangles = new List<Triangle2D>(meshCells.Length);
            for (int t = 0; t < meshCells.Length; ++t)
            {
                DefaultVertex[] verticesOfTriangle = meshCells[t].Vertices;
                Debug.Assert(verticesOfTriangle.Length == 3);
                var triangle = new Triangle2D();
                for (int v = 0; v < 3; ++v)
                {
                    triangle.Vertices[v] = verticesOfTriangle[v].Position;
                }
                triangles.Add(triangle);
            }

            // Remove very small triangles
            if (MinTriangleArea > 0)
            {
                triangles.RemoveAll(tri => tri.CalcArea() < MinTriangleArea);
            }

            return triangles;
        }
    }
}
