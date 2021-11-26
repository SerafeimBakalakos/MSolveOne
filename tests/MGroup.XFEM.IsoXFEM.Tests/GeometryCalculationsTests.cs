namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Matrices;

	using Xunit;

	public class GeometryCalculationsTests
	{
		[Fact]
		public static void PolygonAreaTest()
		{
			var coordinates = Matrix.CreateFromArray(new double[,] { { 20, 20 }, { 15, 10 }, { 20, 5 }, { 30, 10 }, { 35, 15 } });
			int numOfEdges = 5;
			double areaExpected = 162.5;
			double areaComputed = GeometryCalculations.Polygonarea(coordinates, numOfEdges);
			Assert.Equal(areaExpected, areaComputed);
		}

		[Fact]
		public static void MeanTest()
		{
			var coordinates = Matrix.CreateFromArray(new double[,] { { 90, 0 }, { 92.1466978246015, 0 }, { 92.5000000000000, 0.184235651193797 }, { 92.5, 2.5 }, { 90, 2.5 }, { 90, 0 } });
			var coordinatesExpected = Matrix.CreateFromArray(new double[,] { { 91.191116304100250, 0.864039275198966 } });
			var coordinatesComputed = GeometryCalculations.Mean(coordinates);
			for (int i = 0; i < coordinatesExpected.NumColumns; i++)
			{
				Assert.Equal(coordinatesExpected[0, i], coordinatesComputed[0, i], 10);
			}
		}
	}
}
