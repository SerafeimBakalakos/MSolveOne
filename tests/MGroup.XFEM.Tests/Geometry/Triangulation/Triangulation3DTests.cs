using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;
using Xunit;

//TODO: add comment figures
namespace MGroup.XFEM.Tests.Geometry.Triangulation
{
    public static class Triangulation3DTests
    {
        private const string outputDirectory = @"C:\Users\Serafeim\Desktop\HEAT\2020\MeshGen3D\";

        [Fact]
        public static void TestSingleIntersection()
        {
            (List<double[]> points, CellType cellType, double volume) = CreateHexa8();

            var intersections = new List<double[]>[1];
            intersections[0] = new List<double[]>();
            intersections[0].Add(new double[] { -0.75, -1.00, +1.00 });
            intersections[0].Add(new double[] { -0.25, +1.00, +1.00 });
            intersections[0].Add(new double[] { +0.75, +1.00, -1.00 });
            intersections[0].Add(new double[] { +0.25, -1.00, -1.00 });
            points.AddRange(intersections[0]);

            var triangulator = new MIConvexHullTriangulator3D();
            IList<Tetrahedron3D> tetahedra = triangulator.CreateMesh(points);

            //WriteConformingMesh(tetahedra);
            //PlotIntersections(tetahedra, "singleIntersection", intersections);

            var expectedTetrahedra = new List<Tetrahedron3D>();
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, 1, -1 }, new double[] { -0.25, 1, 1 }, 
                new double[] { 0.75, 1, -1 }, new double[] { 0.25, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0.25, -1, -1 }, new double[] { -0.25, 1, 1 }, 
                new double[] { 0.75, 1, -1 }, new double[] { 1, 1, 1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0.25, -1, -1 }, new double[] { 1, 1, 1 }, 
                new double[] { 0.75, 1, -1 }, new double[] { 1, -1, 1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0.25, -1, -1 }, new double[] { -1, 1, -1 }, 
                new double[] { -0.25, 1, 1 }, new double[] { -1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 1, -1, 1 }, new double[] { 1, 1, 1 }, 
                new double[] { -0.25, 1, 1 }, new double[] { 0.25, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0.25, -1, -1 }, new double[] { -0.75, -1, 1 }, 
                new double[] { -0.25, 1, 1 }, new double[] { 1, -1, 1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -0.25, 1, 1 }, new double[] { -1, 1, 1 }, 
                new double[] { -0.75, -1, 1 }, new double[] { -1, 1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 1, -1, -1 }, new double[] { 1, 1, -1 }, 
                new double[] { 1, -1, 1 }, new double[] { 0.75, 1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, -1, -1 }, new double[] { -0.75, -1, 1 }, 
                new double[] { -0.25, 1, 1 }, new double[] { 0.25, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, 1, -1 }, new double[] { -1, -1, 1 }, 
                new double[] { -1, 1, 1 }, new double[] { -0.75, -1, 1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -0.25, 1, 1 }, new double[] { -1, 1, -1 }, 
                new double[] { -0.75, -1, 1 }, new double[] { -1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -0.75, -1, 1 }, new double[] { -1, 1, -1 }, 
                new double[] { -1, -1, 1 }, new double[] { -1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0.25, -1, -1 }, new double[] { 1, -1, 1 }, 
                new double[] { 0.75, 1, -1 }, new double[] { 1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0.75, 1, -1 }, new double[] { 1, -1, 1 }, 
                new double[] { 1, 1, 1 }, new double[] { 1, 1, -1 }));

            double tol = 1E-7;
            Assert.True(TriangulationUtilities.AreEqual(expectedTetrahedra, tetahedra, tol));
        }

        [Fact]
        public static void TestDoubleIntersection()
        {
            (List<double[]> points, CellType cellType, double volume) = CreateHexa8();

            // Intersection 1:
            var intersections = new List<double[]>[2];
            intersections[0] = new List<double[]>();
            intersections[0].Add(new double[] { -0.25, -1.00, +1.00 });
            intersections[0].Add(new double[] { +0.25, +1.00, +1.00 });
            intersections[0].Add(new double[] { +0.75, +1.00, -1.00 });
            intersections[0].Add(new double[] { +0.25, -1.00, -1.00 });
            points.AddRange(intersections[0]);

            // Intersection 2:
            intersections[1] = new List<double[]>();
            intersections[1].Add(new double[] { -1.00, +0.00, +1.00 });
            intersections[1].Add(new double[] { +0.00, +1.00, +1.00 });
            intersections[1].Add(new double[] { -1.00, +1.00, +0.00 });
            points.AddRange(intersections[1]);

            var triangulator = new MIConvexHullTriangulator3D();
            IList<Tetrahedron3D> tetrahedra = triangulator.CreateMesh(points);

            //WriteConformingMesh(tetrahedra);
            //PlotIntersections(tetrahedra, "doubleIntersection", intersections);

            var expectedTetrahedra = new List<Tetrahedron3D>();
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, 0, 1 }, new double[] { -1, -1, 1 },
                new double[] { -0.25, -1, 1 }, new double[] { -1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, -1, -1 }, new double[] { 0.25, -1, -1 },
                new double[] { -1, 0, 1 }, new double[] { -1, 1, 0 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0.25, -1, -1 }, new double[] { 1, -1, 1 },
                new double[] { 0.75, 1, -1 }, new double[] { 1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, 1, 0 }, new double[] { 0.25, -1, -1 },
                new double[] { 0, 1, 1 }, new double[] { 0.75, 1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, 0, 1 }, new double[] { -0.25, -1, 1 },
                new double[] { 0.25, -1, -1 }, new double[] { -1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 1, -1, 1 }, new double[] { 1, 1, 1 },
                new double[] { 0.25, 1, 1 }, new double[] { 0.75, 1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, 1, -1 }, new double[] { 0.75, 1, -1 },
                new double[] { 0.25, -1, -1 }, new double[] { -1, 1, 0 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, -1, -1 }, new double[] { -1, 1, -1 },
                new double[] { 0.25, -1, -1}, new double[] { -1, 1, 0 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 1, -1, -1 }, new double[] { 1, 1, -1 },
                new double[] { 1, 1, 1 }, new double[] { 0.75, 1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, 1, 1 }, new double[] { -1, 0, 1 },
                new double[] { 0, 1, 1 }, new double[] { -1, 1, 0 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0, 1, 1 }, new double[] { 0.75, 1, -1 },
                new double[] { 0.25, -1, -1 }, new double[] { 0.25, 1, 1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, 1, 0 }, new double[] { -1, 0, 1 },
                new double[] { 0, 1, 1 }, new double[] { 0.25, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0, 1, 1 }, new double[] { 0.25, -1, -1 },
                new double[] { -1, 0, 1 }, new double[] { -0.25, -1, 1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0.75, 1, -1 }, new double[] { 1, -1, 1 },
                new double[] { 1, 1, 1 }, new double[] { 1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0.25, -1, -1 }, new double[] { 0.25, 1, 1 },
                new double[] { 0.75, 1, -1 }, new double[] { 1, -1, 1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0, 1, 1 }, new double[] { 0.25, 1, 1 },
                new double[] { 0.25, -1, -1 }, new double[] { -0.25, -1, 1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0.25, -1, -1 }, new double[] { -0.25, -1, 1},
                new double[] { 0.25, 1, 1 }, new double[] { 1, -1, 1 }));

            double tol = 1E-7;
            Assert.True(TriangulationUtilities.AreEqual(expectedTetrahedra, tetrahedra, tol));
        }

        [Fact]
        public static void TestIntersectionThroughNodes()
        {
            (List<double[]> points, CellType cellType, double volume) = CreateHexa8();

            var intersections = new List<double[]>[1];
            intersections[0] = new List<double[]>();
            intersections[0].Add(points[4]);
            intersections[0].Add(points[5]);
            intersections[0].Add(points[2]);
            intersections[0].Add(points[3]);

            var centroid = new double[] { 0, 0, 0 };
            points.Add(centroid);

            var triangulator = new MIConvexHullTriangulator3D();
            IList<Tetrahedron3D> tetrahedra = triangulator.CreateMesh(points);

            //WriteConformingMesh(tetrahedra);
            //PlotIntersections(tetrahedra, "intersectionThroughNodes", intersections);

            var expectedTetrahedra = new List<Tetrahedron3D>();
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0, 0, 0 }, new double[] { 1, 1, -1 },
                new double[] { -1, 1, -1 }, new double[] { -1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, -1, 1 }, new double[] { 1, -1, 1 },
                new double[] { -1, 1, 1 }, new double[] { 0, 0, 0 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, -1, -1 }, new double[] { -1, -1, 1 },
                new double[] { -1, 1, 1 }, new double[] { 0, 0, 0 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0, 0, 0 }, new double[] { -1, -1, 1 },
                new double[] { 1, -1, 1 }, new double[] { -1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 1, 1, -1 }, new double[] { -1, 1, -1 },
                new double[] { -1, 1, 1 }, new double[] { 0, 0, 0 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 1, -1, -1 }, new double[] { 1, 1, -1 },
                new double[] { 1, -1, 1 }, new double[] { 0, 0, 0 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { -1, -1, -1 }, new double[] { 1, -1, -1 },
                new double[] { 1, -1, 1 }, new double[] { 0, 0, 0 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0, 0, 0 }, new double[] { 1, -1, -1 },
                new double[] { 1, 1, -1 }, new double[] { -1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0, 0, 0 }, new double[] { -1, 1, -1 },
                new double[] { -1, 1, 1 }, new double[] { -1, -1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 1, -1, 1 }, new double[] { 1, 1, 1 },
                new double[] { -1, 1, 1 }, new double[] { 0, 0, 0 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0, 0, 0 }, new double[] { 1, 1, 1 },
                new double[] { -1, 1, 1 }, new double[] { 1, 1, -1 }));
            expectedTetrahedra.Add(new Tetrahedron3D(
                new double[] { 0, 0, 0 }, new double[] { 1, -1, 1 },
                new double[] { 1, 1, 1 }, new double[] { 1, 1, -1 }));

            double tol = 1E-7;
            Assert.True(TriangulationUtilities.AreEqual(expectedTetrahedra, tetrahedra, tol));
        }


        private static (List<double[]> points, CellType cellType, double volume) CreateHexa8()
        {
            var points = new List<double[]>();
            points.Add(new double[] { -1, -1, -1});
            points.Add(new double[] { +1, -1, -1});
            points.Add(new double[] { +1, +1, -1});
            points.Add(new double[] { -1, +1, -1});
            points.Add(new double[] { -1, -1, +1});
            points.Add(new double[] { +1, -1, +1});
            points.Add(new double[] { +1, +1, +1});
            points.Add(new double[] { -1, +1, +1 });

            double volume = 8;
            return (points, CellType.Hexa8, volume);
        }

        private static CustomMesh CreateConformingMesh(IList<Tetrahedron3D> tetrahedra)
        {
            var mesh = new CustomMesh();
            foreach (Tetrahedron3D tetra in tetrahedra)
            {
                int startPoint = mesh.NumOutVertices;
                var pointsOfTriangle = new VtkPoint[4];
                for (int v = 0; v < 4; ++v)
                {
                    double[] vertex = tetra.Vertices[v];
                    var point = new VtkPoint(startPoint + v, vertex);
                    pointsOfTriangle[v] = point;
                    mesh.Vertices.Add(point);
                }
                mesh.Cells.Add(new VtkCell(CellType.Tet4, pointsOfTriangle));
            }
            return mesh;
        }

        private static CustomMesh CreateIntersectionMesh(IList<double[]>[] intersections)
        {
            var mesh = new CustomMesh();
            int offset = 0;
            for (int i = 0; i < intersections.Length; ++i)
            {
                IList<double[]> cell = intersections[i];
                CellType cellType;
                if (cell.Count == 3)
                {
                    cellType = CellType.Tri3;
                }
                else if (cell.Count == 4)
                {
                    cellType = CellType.Quad4;
                }
                else throw new NotImplementedException("Unknown intersection shape");

                var verticesOfCell = new List<VtkPoint>();
                for (int v = 0; v < cell.Count; ++v)
                {
                    double[] point = intersections[i][v];
                    var vertex = new VtkPoint(offset + v, point);
                    verticesOfCell.Add(vertex);
                    mesh.Vertices.Add(vertex);
                }
                mesh.Cells.Add(new VtkCell(cellType, verticesOfCell));

                offset += cell.Count;
            }

            return mesh;
        }

        private static CustomMesh CreateOriginalMesh()
        {
            (List<double[]> points, CellType cellType, double volume) = CreateHexa8();
            var mesh = new CustomMesh();
            for (int i = 0; i < points.Count; ++i)
            {
                var point = new VtkPoint(i, points[i]);
                mesh.Vertices.Add(point);
            }
            mesh.Cells.Add(new VtkCell(cellType, mesh.Vertices.ToArray()));
            return mesh;
        }



        private static void PlotIntersections(IList<Tetrahedron3D> tetrahedra, string outputCase,
            List<double[]>[] intersections)
        {
            CustomMesh originalMesh = CreateOriginalMesh();
            string originalMeshPath = outputDirectory + $"{outputCase}_originalMesh.vtk";
            using (var writer = new VtkFileWriter(originalMeshPath))
            {
                writer.WriteMesh(originalMesh);
            }

            CustomMesh intersectionMesh = CreateIntersectionMesh(intersections);
            string intersectionMeshPath = outputDirectory + $"{outputCase}_intersectionMesh.vtk";
            using (var writer = new VtkFileWriter(intersectionMeshPath))
            {
                writer.WriteMesh(intersectionMesh);
            }

            CustomMesh conformingMesh = CreateConformingMesh(tetrahedra);
            string conformingMeshPath = outputDirectory + $"{outputCase}_conformingMesh.vtk";
            using (var writer = new VtkFileWriter(conformingMeshPath))
            {
                writer.WriteMesh(conformingMesh);
            }
        }

        private static void WriteConformingMesh(IList<Tetrahedron3D> tetrahedra)
        {
            var builder = new StringBuilder();
            for (int t = 0; t < tetrahedra.Count; ++t)
            {
                Tetrahedron3D tetra = tetrahedra[t];
                builder.AppendLine($"Tetrahedron {t}: ");
                for (int v = 0; v < tetra.Vertices.Count; ++v)
                {
                    double[] vertex = tetra.Vertices[v];
                    builder.AppendLine($"Vertex {v}: ({vertex[0]}, {vertex[1]}, {vertex[2]})");
                }
                builder.AppendLine();
            }
            Debug.WriteLine(builder.ToString());
        }
    }
}
