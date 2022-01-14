namespace MGroup.XFEM.IsoXFEM.Tests.SolidOnlyTriangulatorTests
{
	using System;
	using System.Collections.Generic;
	using System.Text;
	using System.Linq;

	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;

	using Xunit;

	public class SolidOnlyTriangulator2DTests
	{
		[Fact]
		private void IntersectionPointsTest()
		{
			Matrix elementCoordinates = Matrix.CreateFromArray(new double[,] { { 0, 0 }, { 20, 0 }, { 20, 20 }, { 0, 20 } });
			Vector elementNodalLevelSet = Vector.CreateFromArray(new double[] { 10, -10, -10, 10 });
			Vector elementConnectionExpected = Vector.CreateFromArray(new double[] { 0, 4, 1, 2, 5, 3, 0 });
			Matrix elementCoordinatesWithIntersectionExpected = Matrix.CreateFromArray(new double[,] { { 0, 0 }, { 20, 0 }, { 20, 20 }, { 0, 20 }, { 10, 0 }, { 10, 20 } });
			Vector elementNodalLevelSetWithIntersectionExpected = Vector.CreateFromArray(new double[] { 10, -10, -10, 10, 0, 0 });
			var triangulator = new SolidOnlyTriangulator2D();
			var (elementConnectionComputed, elementCoordinatesWithIntersectionComputed, elementNodalLevelSetWithIntersectionComputed) = triangulator.IntersectionPoints(elementCoordinates, elementNodalLevelSet);
			for (int i = 0; i < elementConnectionExpected.Length; i++)
			{
				Assert.Equal(elementConnectionExpected[i], elementConnectionComputed[i]);
			}
			for (int i = 0; i < elementCoordinatesWithIntersectionExpected.NumRows; i++)
			{
				for (int j = 0; j < elementCoordinatesWithIntersectionExpected.NumColumns; j++)
				{
					Assert.Equal(elementCoordinatesWithIntersectionExpected[i, j], elementCoordinatesWithIntersectionComputed[i, j]);
				}
			}
			for (int i = 0; i < elementNodalLevelSetWithIntersectionExpected.Length; i++)
			{
				Assert.Equal(elementNodalLevelSetWithIntersectionExpected[i], elementNodalLevelSetWithIntersectionComputed[i]);
			}
		}
		[Fact]
		private void CoordinatesNodesofXFEMpointsTest()
		{
			Vector elementConnection = Vector.CreateFromArray(new double[] { 0, 4, 1, 2, 5, 3, 0 });
			Matrix elementCoordinatesWithIntersection = Matrix.CreateFromArray(new double[,] { { -1, -1 }, { +1, -1 }, { +1, +1 }, { -1, +1 }, { 0, -1 }, { 0, +1 } });
			Vector elementNodalLevelSetWithIntersection = Vector.CreateFromArray(new double[] { 10, -10, -10, 10, 0, 0 });
			Matrix elementCoordinatesWithIntersectionAndCentrePointsExpected = Matrix.CreateFromArray(new double[,] { { -1, -1 }, { +1, -1 }, { +1, +1 }, { -1, +1 }, { 0, -1 }, { 0, +1 }, { -0.5, 0 } });
			int[] connectionCircularNoNegativeNodesWithIntersectionExpected = new int[] { 0, 4, 5, 3, 0 };
			var triangulator = new SolidOnlyTriangulator2D();
			var (elementCoordinatesWithIntersectionAndCentrePointsComputed, connectionCircularNoNegativeNodesWithIntersectionComputed) = triangulator.CoordinatesNodesofXFEMpoints(elementConnection, elementCoordinatesWithIntersection, elementNodalLevelSetWithIntersection);
			for (int i = 0; i < elementCoordinatesWithIntersectionAndCentrePointsExpected.NumRows; i++)
			{
				for (int j = 0; j < elementCoordinatesWithIntersectionAndCentrePointsExpected.NumColumns; j++)
				{
					Assert.Equal(elementCoordinatesWithIntersectionAndCentrePointsExpected[i, j], elementCoordinatesWithIntersectionAndCentrePointsComputed[i, j]);
				}
			}
			for (int i = 0; i < connectionCircularNoNegativeNodesWithIntersectionExpected.Length; i++)
			{
				Assert.Equal(connectionCircularNoNegativeNodesWithIntersectionExpected[i], connectionCircularNoNegativeNodesWithIntersectionComputed[i]);
			}
		}
		[Fact]
		private void ConnectionOfSubTrianglesTest()
		{
			Matrix elementCoordinatesWithIntersectionAndCentrePoints = Matrix.CreateFromArray(new double[,] { { -1, -1 }, { +1, -1 }, { +1, +1 }, { -1, +1 }, { 0, -1 }, { 0, +1 }, { -0.5, 0 } });
			int[] connectionCircularNoNegativeNodesWithIntersection = new int[] { 0, 4, 5, 3, 0 };
			int[,] trianlesConnectionExpected = new int[,] { { 0, 4, 6 }, { 4, 5, 6 }, { 5, 3, 6 }, { 3, 0, 6 } };
			var triangulator = new SolidOnlyTriangulator2D();
			var trianlesConnectionComputed = triangulator.ConnectionOfSubTriangles(connectionCircularNoNegativeNodesWithIntersection, elementCoordinatesWithIntersectionAndCentrePoints);
			for (int i = 0; i < trianlesConnectionExpected.GetLength(0); i++)
			{
				for (int j = 0; j < trianlesConnectionExpected.GetLength(1); j++)
				{
					Assert.Equal(trianlesConnectionExpected[i, j], trianlesConnectionComputed[i, j]);
				}
			}
		}
		[Fact]
		private void CreateSubTrianglesOfElementTest()
		{
			//For Example:
			//                  LevelSet                                     
			//                      |
			//      #3           #5 |               #2                            
			//       .______________._______________.                                                   
			//       |\            /|               |                                                  
			//       | \          / |               |                                                  
			//       |  \   +    /  |        -      |
			//       |   \      /   |               |                              
			//       |    \    /    |               |
			//       |     \  /     |               |
			//       |      #6.     |               |
			//       |      / \     |               |
			//       |     /   \    |               |
			//       |    /     \   |               |
			//       |   /       \  |               |
			//       |  /         \ |               |
			//       | /           \|               |
			//     #0.____________#4.______________#1.
			//                      |
			//                      |
			Matrix coordinatesOfElement = Matrix.CreateFromArray(new double[,] { { 0, 0 }, { 10, 0 }, { 10, 10 }, { 0, 10 } });
			Matrix elementCoordinatesWithIntersectionPoints = Matrix.CreateFromArray(new double[,] { { 0, 0 }, { 10, 0 }, { 10, 10 }, { 0, 10 }, { 5, 0 }, { 5, 10 }, { 2.5, 5 } });
			int[,] trianglesConnection = new int[,] { { 0, 4, 6 }, { 4, 5, 6 }, { 5, 3, 6 }, { 3, 0, 6 } };
			var triangulator = new SolidOnlyTriangulator2D();
			var triangles2DComputed = triangulator.CreateSubTrianglesOfElement(coordinatesOfElement, elementCoordinatesWithIntersectionPoints, trianglesConnection);
			List<IList<double[]>> verticesNaturalOfSubTriangles = new List<IList<double[]>>();
			IList<double[]> verticesNatural1stTriangle = new List<double[]>();
			var point0 = new double[] { -1, -1 };
			var point4 = new double[] { 0, -1 };
			var point6 = new double[] { -0.5, 0 };
			var point5 = new double[] { 0, 1 };
			var point3 = new double[] { -1, 1 };
			verticesNatural1stTriangle.Add(point0);
			verticesNatural1stTriangle.Add(point4);
			verticesNatural1stTriangle.Add(point6);
			IList<double[]> verticesNatural2ndTriangle = new List<double[]>();
			verticesNatural2ndTriangle.Add(point4);
			verticesNatural2ndTriangle.Add(point5);
			verticesNatural2ndTriangle.Add(point6);
			IList<double[]> verticesNatural3rdTriangle = new List<double[]>();
			verticesNatural3rdTriangle.Add(point5);
			verticesNatural3rdTriangle.Add(point3);
			verticesNatural3rdTriangle.Add(point6);
			IList<double[]> verticesNatural4thTriangle = new List<double[]>();
			verticesNatural4thTriangle.Add(point3);
			verticesNatural4thTriangle.Add(point0);
			verticesNatural4thTriangle.Add(point6);
			verticesNaturalOfSubTriangles.Add(verticesNatural1stTriangle);
			verticesNaturalOfSubTriangles.Add(verticesNatural2ndTriangle);
			verticesNaturalOfSubTriangles.Add(verticesNatural3rdTriangle);
			verticesNaturalOfSubTriangles.Add(verticesNatural4thTriangle);
			int m = 0;
			foreach (var triangle in triangles2DComputed)
			{
				var verticesNaturalComputed = triangle.VerticesNatural;
				int v = 0;
				foreach( var vertices in verticesNaturalComputed)
				{
					Assert.Equal(verticesNaturalOfSubTriangles[m][v][0], vertices[0]);
					Assert.Equal(verticesNaturalOfSubTriangles[m][v][1], vertices[1]);
					v++;
				}
				m++;
			}

		}

	}
}
