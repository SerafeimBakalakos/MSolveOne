using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Geometry.Mesh;
using Xunit;

namespace MGroup.XFEM.Tests.Geometry.DualMesh
{
	public class DualCartesianSimplicialSymmetricMesh3DTests
	{
		[Fact]
		public static void TestCoordinateMappings()
		{
			double[] minCoords = { -1, -1, -1 };
			double[] maxCoords = { +1, +1, +1 };
			int[] numNodesCoarse = { 2, 2, 2 };
			int[] numNodesFine = { 3, 3, 3 };

			var mesh = new DualCartesianSimplicialSymmetricMesh3D.Builder(minCoords, maxCoords, numNodesCoarse, numNodesFine)
					.BuildMesh();
			foreach ((int elementID, int[] nodeIDs) in mesh.FineMesh.EnumerateElements())
			{
				Assert.Equal(4, nodeIDs.Length);
				int subtetrahedron = elementID;
				int[] elementIdx = mesh.FineMesh.GetElementIdx(elementID);

				var verticesNaturalCoarse = new double[nodeIDs.Length][];
				for (int n = 0; n < nodeIDs.Length; ++n)
				{
					verticesNaturalCoarse[n] = mesh.FineMesh.GetNodeCoordinates(nodeIDs[n]);
				}
				List<double[]> pointsNaturalCoarse = GeneratePointsInTetrahedron(verticesNaturalCoarse);

				var verticesNaturalFine = new double[][]
				{
					new double[] { 0, 0, 0 },
					new double[] { 1, 0, 0 },
					new double[] { 0, 1, 0 },
					new double[] { 0, 0, 1 },
				};
				List<double[]> pointsNaturalFine = GeneratePointsInTetrahedron(verticesNaturalFine);

				double tolerance = 1E-10;
				for (int i = 0; i < pointsNaturalFine.Count; ++i)
				{
					var coordsNaturalCoarseExpected = Vector.CreateFromArray(pointsNaturalCoarse[i]);
					var coordsNaturalFineExpected = Vector.CreateFromArray(pointsNaturalFine[i]);

					var coordsNaturalCoarseComputed = Vector.CreateFromArray(
						mesh.MapPointFineNaturalToCoarseNatural(elementIdx, pointsNaturalFine[i]));
					var coordsNaturalFineComputed = Vector.CreateFromArray(
						mesh.MapPointCoarseNaturalToFineNatural(subtetrahedron, pointsNaturalCoarse[i]));

					Assert.True(coordsNaturalCoarseExpected.Equals(coordsNaturalCoarseComputed, tolerance));
					Assert.True(coordsNaturalFineExpected.Equals(coordsNaturalFineComputed, tolerance));
				}
			}
		}

		[Fact]
		public static void TestIdentificationOfSubtetrahedron()
		{
			double[] minCoords = { -1, -1, -1 };
			double[] maxCoords = { +1, +1, +1 };
			int[] numNodesCoarse = { 2, 2, 2 };
			int[] numNodesFine = { 3, 3, 3 };

			var mesh = new DualCartesianSimplicialSymmetricMesh3D.Builder(minCoords, maxCoords, numNodesCoarse, numNodesFine)
					.BuildMesh();
			foreach ((int elementID, int[] nodeIDs) in mesh.FineMesh.EnumerateElements())
			{
				int subtetrahedron = elementID;
				Assert.Equal(4, nodeIDs.Length);

				var vertices = new double[nodeIDs.Length][];
				for (int n = 0; n < nodeIDs.Length; ++n)
				{
					vertices[n] = mesh.FineMesh.GetNodeCoordinates(nodeIDs[n]);
				}

				List<double[]> internalPoints = GeneratePointsInTetrahedron(vertices);
				foreach (double[] point in internalPoints)
				{
					int subtetrahedronComputed = mesh.FindSubcellIdx(point);
					Assert.Equal(subtetrahedron, subtetrahedronComputed);

				}
			}
		}

		private static double[] FindCentroid(params double[][] vertices)
		{
			int numVertices = vertices.Length;
			int dim = vertices[0].Length;
			var centroid = new double[dim];
			foreach (double[] vertex in vertices)
			{
				for (int d = 0; d < dim; ++d)
				{
					centroid[d] += vertex[d];
				}
			}
			for (int d = 0; d < dim; ++d)
			{
				centroid[d] /= numVertices;
			}
			return centroid;
		}

		private static List<double[]> GeneratePointsInTetrahedron(double[][] vertices)
		{
			var points = new List<double[]>();

			// First the centroid
			double[] centroid = FindCentroid(vertices);
			points.Add(centroid);

			// Seperate the tetrahedron into subtetrahedra and take the centroids of those
			points.Add(FindCentroid(vertices[0], vertices[1], vertices[2], centroid));
			points.Add(FindCentroid(vertices[0], vertices[1], vertices[3], centroid));
			points.Add(FindCentroid(vertices[0], vertices[2], vertices[3], centroid));
			points.Add(FindCentroid(vertices[1], vertices[2], vertices[3], centroid));

			// Offset vertices halfway towards centroid
			points.Add(OffsetPointHalfwayTowards(vertices[0], centroid));
			points.Add(OffsetPointHalfwayTowards(vertices[1], centroid));
			points.Add(OffsetPointHalfwayTowards(vertices[2], centroid));
			points.Add(OffsetPointHalfwayTowards(vertices[3], centroid));

			return points;
		}

		private static double[] OffsetPointHalfwayTowards(double[] start, double[] destination)
		{
			double move = 0.5;
			int dim = start.Length;
			var result = new double[dim];
			for (int d = 0; d < dim; ++d)
			{
				result[d] = start[d] + move * (destination[d] - start[d]);
			}
			return result;
		}
	}
}
