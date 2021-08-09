using System;
using System.Collections.Generic;
using System.Diagnostics;
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
	public static class Triangulation2DTests
	{
		private const string outputDirectory = @"C:\Users\Serafeim\Desktop\HEAT\2020\MeshGen\";

		[Fact]
		public static void TestSingleIntersection()
		{
			(List<double[]> points, CellType cellType) = CreatePolygon();
			double outlineArea = TriangulationUtilities.CalcPolygonArea(points);

			var intersections = new List<double[]>[1];
			intersections[0] = new List<double[]>();
			intersections[0].Add(new double[] { 1.95, 3.505 });
			intersections[0].Add(new double[] { 1.7, 0.3 });
			points.AddRange(intersections[0]);

			var triangulator = new MIConvexHullTriangulator2D();
			triangulator.MinTriangleArea = 1E-5 * outlineArea;
			IList<Triangle2D> triangles = triangulator.CreateMesh(points);

			//WriteConformingMesh(triangles);
			PlotIntersections(triangles, "singleIntersection", intersections);

			var expectedTriangles = new List<Triangle2D>();
			expectedTriangles.Add(new Triangle2D(
				new double[] { 1.95, 3.505 }, new double[] { 0, 2 }, new double[] { 1, 4 }));
			expectedTriangles.Add(new Triangle2D(
				new double[] { 0, 2 }, new double[] { 1.95, 3.505 }, new double[] { 1.7, 0.3 }));
			expectedTriangles.Add(new Triangle2D(
				new double[] { 1.7, 0.3 }, new double[] { 1.95, 3.505 }, new double[] { 5, 1.8 }));
			expectedTriangles.Add(new Triangle2D(
				new double[] { 1.7, 0.3 }, new double[] { 5, 1.8 }, new double[] { 2, 0 }));

			double tol = 1E-7;
			Assert.True(TriangulationUtilities.AreEqual(expectedTriangles, triangles, tol));
		}

		[Fact]
		public static void TestDoubleIntersection()
		{
			(List<double[]> points, CellType cellType) = CreatePolygon();
			double outlineArea = TriangulationUtilities.CalcPolygonArea(points);

			// Intersection 1:
			var intersections = new List<double[]>[2];
			intersections[0] = new List<double[]>();
			intersections[0].Add(new double[] { 0.4, 1.6 });
			intersections[0].Add(new double[] { 2.75, 3.0375 });
			points.AddRange(intersections[0]);

			// Intersection 2:
			intersections[1] = new List<double[]>();
			intersections[1].Add(new double[] { 1.6, 0.4 });
			intersections[1].Add(new double[] { 3.5, 2.625 });
			points.AddRange(intersections[1]);

			var triangulator = new MIConvexHullTriangulator2D();
			triangulator.MinTriangleArea = 1E-5 * outlineArea;
			IList<Triangle2D> triangles = triangulator.CreateMesh(points);

			//WriteConformingMesh(triangles);
			PlotIntersections(triangles, "doubleIntersection", intersections);

			var expectedTriangles = new List<Triangle2D>();
			expectedTriangles.Add(new Triangle2D(
				new double[] { 0.4, 1.6 }, new double[] { 0, 2 }, new double[] { 1, 4 }));
			expectedTriangles.Add(new Triangle2D(
				new double[] { 2.75, 3.0375 }, new double[] { 0.4, 1.6 }, new double[] { 1, 4 }));
			expectedTriangles.Add(new Triangle2D(
				new double[] { 0.4, 1.6 }, new double[] { 2.75, 3.0375 }, new double[] { 1.6, 0.4 }));
			expectedTriangles.Add(new Triangle2D(
				new double[] { 3.5, 2.625 }, new double[] { 1.6, 0.4 }, new double[] { 2.75, 3.0375 }));
			expectedTriangles.Add(new Triangle2D(
				new double[] { 2, 0 }, new double[] { 1.6, 0.4 }, new double[] { 3.5, 2.625 }));
			expectedTriangles.Add(new Triangle2D(
				new double[] { 3.5, 2.625 }, new double[] { 5, 1.8 }, new double[] { 2, 0 }));


			double tol = 1E-7;
			Assert.True(TriangulationUtilities.AreEqual(expectedTriangles, triangles, tol));
		}

		[Fact]
		public static void TestIntersectionThroughNodes()
		{
			(List<double[]> points, CellType cellType) = CreatePolygon();
			double outlineArea = TriangulationUtilities.CalcPolygonArea(points);

			var intersections = new List<double[]>[1];
			intersections[0] = new List<double[]>();
			intersections[0].Add(points[1]);
			intersections[0].Add(points[3]);

			var middle = new double[] { 0.5 * (points[1][0] + points[3][0]), 0.5 * (points[1][1] + points[3][1]) };
			points.Add(middle);

			var triangulator = new MIConvexHullTriangulator2D();
			triangulator.MinTriangleArea = 1E-5 * outlineArea;
			IList<Triangle2D> triangles = triangulator.CreateMesh(points);

			//WriteConformingMesh(triangles);
			PlotIntersections(triangles, "intersectionThroughNodes", intersections);

			var expectedTriangles = new List<Triangle2D>();
			expectedTriangles.Add(new Triangle2D(
				new double[] { 2.5, 1.9 }, new double[] { 0, 2 }, new double[] { 1, 4 }));
			expectedTriangles.Add(new Triangle2D(
				new double[] { 2, 0 }, new double[] { 0, 2 }, new double[] { 2.5, 1.9 }));
			expectedTriangles.Add(new Triangle2D(
				new double[] { 2.5, 1.9 }, new double[] { 5, 1.8 }, new double[] { 2, 0 }));
			expectedTriangles.Add(new Triangle2D(
				new double[] { 2.5, 1.9 }, new double[] { 1, 4 }, new double[] { 5, 1.8 }));

			double tol = 1E-7;
			Assert.True(TriangulationUtilities.AreEqual(expectedTriangles, triangles, tol));
		}
		

		private static (List<double[]> points, CellType cellType) CreatePolygon()
		{
			var points = new List<double[]>();
			points.Add(new double[] { 2, 0 });
			points.Add(new double[] { 5, 1.8 });
			points.Add(new double[] { 1, 4 });
			points.Add(new double[] { 0, 2 });
			return (points, CellType.Quad4);
		}

		private static CustomMesh CreateConformingMesh(IList<Triangle2D> triangles)
		{
			var mesh = new CustomMesh();
			foreach (Triangle2D triangle in triangles)
			{
				int startPoint = mesh.NumOutVertices;
				var pointsOfTriangle = new VtkPoint[3];
				for (int v = 0; v < 3; ++v)
				{
					double[] vertex = triangle.Vertices[v];
					var point = new VtkPoint(startPoint + v, vertex);
					pointsOfTriangle[v] = point;
					mesh.Vertices[point.ID] = point;
				}
				mesh.Cells.Add(new VtkCell(CellType.Tri3, pointsOfTriangle));
			}
			return mesh;
		}

		private static CustomMesh CreateIntersectionMesh(IList<double[]>[] intersections)
		{
			var mesh = new CustomMesh();
			int offset = 0;
			for (int i = 0; i < intersections.Length; ++i)
			{
				for (int v = 0; v < intersections[i].Count; ++v)
				{
					double[] point = intersections[i][v];
					int id = offset + v;
					mesh.Vertices[id] = new VtkPoint(id, point);
				}

				for (int c = 0; c < intersections[i].Count - 1; ++c)
				{
					var pointsOfCell = new VtkPoint[] { mesh.Vertices[offset + c], mesh.Vertices[offset + c + 1] };
					mesh.Cells.Add(new VtkCell(CellType.Line2, pointsOfCell));
				}

				offset += intersections[i].Count;
			}

			return mesh;
		}

		private static CustomMesh CreateOriginalMesh()
		{
			(List<double[]> points, CellType cellType) = CreatePolygon();
			var mesh = new CustomMesh();
			var cellVertices = new List<VtkPoint>();
			for (int i = 0; i < points.Count; ++i)
			{
				var point = new VtkPoint(i, points[i]);
				mesh.Vertices[point.ID] = point;
				cellVertices.Add(point);
			}
			mesh.Cells.Add(new VtkCell(cellType, cellVertices));
			return mesh;
		}

		

		private static void PlotIntersections(IList<Triangle2D> triangles, string outputCase,
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

			CustomMesh conformingMesh = CreateConformingMesh(triangles);
			string conformingMeshPath = outputDirectory + $"{outputCase}_conformingMesh.vtk";
			using (var writer = new VtkFileWriter(conformingMeshPath))
			{
				writer.WriteMesh(conformingMesh);
			}
		}

		private static void WriteConformingMesh(IList<Triangle2D> triangles)
		{
			var builder = new StringBuilder();
			for (int t = 0; t < triangles.Count; ++t)
			{
				Triangle2D triangle = triangles[t];
				builder.AppendLine($"Triangle {t}: ");
				for (int v = 0; v < triangle.Vertices.Count; ++v)
				{
					double[] vertex = triangle.Vertices[v];
					builder.AppendLine($"Vertex {v}: ({vertex[0]}, {vertex[1]})");
				}
				builder.AppendLine();
			}
			Debug.WriteLine(builder.ToString());
		}
	}
}
