namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System;
	using System.Collections.Generic;
	using System.Text;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.LinearAlgebra.Matrices;

	using Xunit;

	public class ElementTests
	{
		[Fact]
		public void ElementTest()
		{
			var geometry = new GeometryProperties(40, 40, 1, 2, 2);
			var material = new MaterialProperties(1, 0.3);
			var nodes = new List<Node>();
			nodes.Add(new Node(0, 0, 0, true, true));
			nodes.Add(new Node(1, 0, 20, true, true));
			nodes.Add(new Node(2, 0, 40, true, true));
			nodes.Add(new Node(3, 20, 0, false, false));
			nodes.Add(new Node(4, 20, 20, false, false));
			nodes.Add(new Node(5, 20, 40, false, false));
			nodes.Add(new Node(6, 40, 0, false, false));
			nodes.Add(new Node(7, 40, 20, false, false));
			nodes.Add(new Node(8, 40, 40, false, false));
			var element = new Element(0, material, geometry, new[]
			{
				nodes[0],
				nodes[3],
				nodes[4],
				nodes[1]
			});
			var coordinatesOfElementExpected = Matrix.CreateFromArray(new double[,] { { 0, 0 }, { 20, 0 }, { 20, 20 }, { 0, 20 } });
			var areaOfElementExpected = 400.00;
			var stiffnessOfElementExpected = Matrix.CreateFromArray(new double[,] { { 0.494505494505, 0.178571428571, - 0.302197802198, - 0.0137362637363, - 0.247252747253, - 0.178571428571, 0.0549450549451, 0.0137362637363 },
			{ 0.178571428571,  0.494505494505,  0.0137362637363, 0.0549450549451, -0.178571428571, -0.247252747253, -0.0137362637363,  -0.302197802198},
			{ -0.302197802198, 0.0137362637363, 0.494505494505,  -0.178571428571, 0.0549450549451, -0.0137362637363,    -0.247252747253, 0.178571428571},
			{-0.0137362637363,   0.0549450549451, -0.178571428571, 0.494505494505,  0.0137362637363, -0.302197802198, 0.178571428571,  -0.247252747253 },
			{-0.247252747253,    -0.178571428571, 0.0549450549451, 0.0137362637363, 0.494505494505,  0.178571428571,  -0.302197802198, -0.0137362637363 },
			{-0.178571428571, -0.247252747253, -0.0137362637363,    -0.302197802198, 0.178571428571,  0.494505494505,  0.0137362637363, 0.0549450549451 },
			{ 0.0549450549451,   -0.0137362637363,  -0.247252747253, 0.178571428571,  -0.302197802198, 0.0137362637363, 0.494505494505,  -0.178571428571},
			{0.0137362637363,  -0.302197802198, 0.178571428571,  -0.247252747253, -0.0137362637363,    0.0549450549451, -0.178571428571, 0.494505494505 } });
			Matrix coordinatesOfElementComputed = element.coordinatesOfElement;
			double areaOfElementComputed = element.areaOfElement;
			Matrix stiffnessOfElementComputed = element.stiffnessOfElement;
			Assert.Equal(areaOfElementExpected, areaOfElementComputed);
			for (int i = 0; i < coordinatesOfElementExpected.NumRows; i++)
			{
				for (int j = 0; j < coordinatesOfElementExpected.NumColumns; j++)
				{
					Assert.Equal(coordinatesOfElementExpected[i, j], coordinatesOfElementComputed[i, j]);
				}
			}
			for (int i = 0; i < stiffnessOfElementExpected.NumRows; i++)
			{
				for (int j = 0; j < stiffnessOfElementExpected.NumColumns; j++)
				{
					Assert.Equal(stiffnessOfElementExpected[i, j], stiffnessOfElementComputed[i, j],8);
				}
			}
		}
		[Fact]
		public void CalcStiffnesAndAreaTest()
		{
			var geometry = new GeometryProperties(40, 40, 1, 2, 2);
			var material = new MaterialProperties(1, 0.3);
			var element = new Element(0, material, geometry, new[]
			{
				new Node(0, 0, 0, true, true),
				new Node(3, 20, 0, false, false),
				new Node(4, 20, 20, false, false),
				new Node(1, 0, 20, true, true)
			});
			Vector elementLevelSet = Vector.CreateFromArray(new double[] { 10, -10, -10, 10 });
			element.CalcStiffnessAndArea(elementLevelSet);
			var areaOfElementExpected = 200.00;
			var stiffnessOfElementExpected= Matrix.CreateFromArray(new double[,] { { 0.29532967033, 0.133928571429, - 0.151098901099, - 0.051510989011, - 0.123626373626, - 0.0927197802198, - 0.0206043956044,    0.0103021978022},
			{0.133928571429, 0.384615384615,  -0.0377747252747, 0.0274725274725, -0.0858516483516, -0.123626373626, -0.0103021978022, -0.288461538462 },
			{ -0.151098901099,   -0.0377747252747,    0.199175824176,  -0.0446428571429, 0.0755494505495, -0.00343406593407,   -0.123626373626, 0.0858516483516},
			{ -0.051510989011, 0.0274725274725, -0.0446428571429, 0.10989010989,   0.00343406593407, -0.0137362637363,  0.0927197802198, -0.123626373626},
			{ -0.123626373626,   -0.0858516483516, 0.0755494505495, 0.00343406593407,  0.199175824176,  0.0446428571429, -0.151098901099, 0.0377747252747},
			{ -0.0927197802198,  -0.123626373626, -0.00343406593407, -0.0137362637363,    0.0446428571429, 0.10989010989,   0.051510989011,  0.0274725274725},
			{ -0.0206043956044,  -0.0103021978022,  -0.123626373626, 0.0927197802198, -0.151098901099, 0.051510989011,  0.29532967033,   -0.133928571429},
			{ 0.0103021978022, -0.288461538462, 0.0858516483516, -0.123626373626, 0.0377747252747, 0.0274725274725, -0.133928571429, 0.384615384615 }});
			var areaOfElementComputed = element.areaOfElement;
			var stiffnessOfElementComputed = element.stiffnessOfElement;
			Assert.Equal(areaOfElementExpected, areaOfElementComputed);
			for (int i = 0; i < stiffnessOfElementExpected.NumRows; i++)
			{
				for (int j = 0; j < stiffnessOfElementExpected.NumColumns; j++)
				{
					Assert.Equal(stiffnessOfElementExpected[i, j], stiffnessOfElementComputed[i, j],10);
				}
			}
		}
	}
}
