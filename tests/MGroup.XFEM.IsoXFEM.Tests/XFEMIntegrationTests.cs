namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System;
	using System.Collections.Generic;
	using System.Text;
	using MGroup.XFEM.IsoXFEM;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using Xunit;

	public class XFEMIntegrationTests
	{
		[Fact]
		private void IntersectionPointsTest()
		{
			Matrix elementCoordinates = Matrix.CreateFromArray(new double[,] { { 0, 0 }, { 20, 0 }, { 20, 20 }, { 0, 20 } });
			Vector elementNodalLevelSet = Vector.CreateFromArray(new double[] { 10, -10, -10, 10 });
			Vector elementConnectionExpected = Vector.CreateFromArray(new double[] { 0, 4, 1, 2, 5, 3, 0 });
			Matrix elementCoordinatesWithIntersectionExpected = Matrix.CreateFromArray(new double[,] { { 0, 0 }, { 20, 0 }, { 20, 20 }, { 0, 20 }, { 10, 0 }, { 10, 20 } });
			Vector elementNodalLevelSetWithIntersectionExpected = Vector.CreateFromArray(new double[] { 10, -10, -10, 10, 0, 0 });
			var integration = new XFEMIntegration();
			var (elementConnectionComputed, elementCoordinatesWithIntersectionComputed, elementNodalLevelSetWithIntersectionComputed) = integration.IntersectionPoints(elementCoordinates, elementNodalLevelSet);
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
			var integration = new XFEMIntegration();
			var (elementCoordinatesWithIntersectionAndCentrePointsComputed, connectionCircularNoNegativeNodesWithIntersectionComputed) = integration.CoordinatesNodesofXFEMpoints(elementConnection, elementCoordinatesWithIntersection, elementNodalLevelSetWithIntersection);
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
		private void ConnectionOfSubTrianglesAndAreaSubElementTest()
		{
			Matrix elementCoordinatesWithIntersectionAndCentrePoints = Matrix.CreateFromArray(new double[,] { { -1, -1 }, { +1, -1 }, { +1, +1 }, { -1, +1 }, { 0, -1 }, { 0, +1 }, { -0.5, 0 } });
			int[] connectionCircularNoNegativeNodesWithIntersection = new int[] { 0, 4, 5, 3, 0 };
			double areaOfsubElementExpected = 2.00;
			int[,] trianlesConnectionExpected = new int[,] { { 0, 4, 6 }, { 4, 5, 6 }, { 5, 3, 6 }, { 3, 0, 6 } };
			var integration = new XFEMIntegration();
			var (trianlesConnectionComputed, areaOfsubElementComputed) = integration.ConnectionOfSubTrianglesAndAreaSubElement(connectionCircularNoNegativeNodesWithIntersection, elementCoordinatesWithIntersectionAndCentrePoints);
			Assert.Equal(areaOfsubElementExpected, areaOfsubElementComputed);
			for (int i = 0; i < trianlesConnectionExpected.GetLength(0); i++)
			{
				for (int j = 0; j < trianlesConnectionExpected.GetLength(1); j++)
				{
					Assert.Equal(trianlesConnectionExpected[i, j], trianlesConnectionComputed[i, j]);
				}
			}
		}

		[Fact]
		public void MeshAndAreaOfSubElementTest()
		{
			var coordinatesOfElement = Matrix.CreateFromArray(new double[,] { { -1, -1 }, { 1, -1 }, { 1, 1 }, { -1, 1 } });
			var elementLevelSet = Vector.CreateFromArray(new double[] { 10, -10, -10, 10 });
			var integration = new XFEMIntegration();
			integration.MeshAndAreaOfSubElement(coordinatesOfElement, elementLevelSet);
			var areaOfElementExpected = 2.00;
			var connectionOfElementExpected = new int[,] { { 0, 4, 6 }, { 4, 5, 6 }, { 5, 3, 6 }, { 3, 0, 6 } };
			var coordinatesOfElementExpected = Matrix.CreateFromArray(new double[,] { { -1, -1 }, { +1, -1 }, { +1, +1 }, { -1, +1 }, { 0, -1 }, { 0, +1 }, { -0.5, 0 } });
			var areaOfElementComputed = integration.areaBoundaryElement;
			var connectionOfElementComputed = integration.connectionOfBoundaryElement;
			var coordinatesOfElementComputed = integration.coordinatesOfBoundaryElement;
			Assert.Equal(areaOfElementExpected, areaOfElementComputed);
			Assert.Equal(connectionOfElementExpected.GetLength(0), connectionOfElementComputed.GetLength(0));
			Assert.Equal(connectionOfElementExpected.GetLength(1), connectionOfElementComputed.GetLength(1));
			Assert.Equal(coordinatesOfElementExpected.NumRows, coordinatesOfElementComputed.NumRows);
			Assert.Equal(coordinatesOfElementExpected.NumColumns, coordinatesOfElementComputed.NumColumns);
			for (int i = 0; i < connectionOfElementExpected.GetLength(0); i++)
			{
				for (int j = 0; j < connectionOfElementExpected.GetLength(1); j++)
				{
					Assert.Equal(connectionOfElementExpected[i, j], connectionOfElementComputed[i, j]);
				}
			}
			for (int i = 0; i < coordinatesOfElementExpected.NumRows; i++)
			{
				for (int j = 0; j < coordinatesOfElementExpected.NumColumns; j++)
				{
					Assert.Equal(coordinatesOfElementExpected[i, j], coordinatesOfElementComputed[i, j]);
				}
			}

		}
	}
}
