namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System;
	using System.Collections.Generic;
	using System.Text;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.LinearAlgebra.Matrices;

	using Xunit;
	using MGroup.XFEM.Entities;
	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.XFEM.Materials.Duplicates;

	public class IsoXfemElement2DTests
	{
		[Fact]
		public void IsoXfemElement2DTest()
		{
			var geometry = new GeometryProperties(40, 40, 1, 2, 2);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var nodes = new List<XNode>();
			nodes.Add(new XNode(0, new double[] { 0, 0 } ));
			nodes.Add(new XNode(1, new double[] { 0, 20 } ));
			nodes.Add(new XNode(2, new double[] { 0, 40 } ));
			nodes.Add(new XNode(3, new double[] { 20, 0 } ));
			nodes.Add(new XNode(4, new double[] { 20, 20 }));
			nodes.Add(new XNode(5, new double[] { 20, 40 }));
			nodes.Add(new XNode(6, new double[] { 40, 0 }));
			nodes.Add(new XNode(7, new double[] { 40, 20 }));
			nodes.Add(new XNode(8, new double[] { 40, 40 }));
			nodes[0].Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
			nodes[0].Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
			nodes[1].Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
			nodes[1].Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
			nodes[2].Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
			nodes[2].Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
			var element = new IsoXfemElement2D(0, material, geometry, new[]
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
			Matrix coordinatesOfElementComputed = element.CoordinatesOfElement;
			double areaOfElementComputed = element.AreaOfElement;
			Matrix stiffnessOfElementComputed = element.StiffnessOfElement;
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
			var Kstnd = element.BuildStiffnessMatrixStandard();
			for (int i = 0; i < stiffnessOfElementExpected.NumRows; i++)
			{
				for (int j = 0; j < stiffnessOfElementExpected.NumColumns; j++)
				{
					Assert.Equal(stiffnessOfElementExpected[i, j], Kstnd[i, j], 10);
				}
			}
		}
		[Fact]
		public void CalcStiffnesAndAreaTest()
		{
			var geometry = new GeometryProperties(40, 40, 1, 2, 2);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var element = new IsoXfemElement2D(0, material, geometry, new[]
			{
				new XNode(0, new double[] { 0, 0 } ),
				new XNode(3, new double[] { 20, 0 } ),
				new XNode(4, new double[] { 20, 20 }),
				new XNode(1, new double[] { 0, 20 } )
			});
			element.ElementLevelSet = Vector.CreateFromArray(new double[] { 10, -10, -10, 10 });
			element.StiffnessMatrix(element);
			var areaOfElementExpected = 200.00;
			var stiffnessOfElementExpected= Matrix.CreateFromArray(new double[,] { { 0.29532967033, 0.133928571429, - 0.151098901099, - 0.051510989011, - 0.123626373626, - 0.0927197802198, - 0.0206043956044,    0.0103021978022},
			{0.133928571429, 0.384615384615,  -0.0377747252747, 0.0274725274725, -0.0858516483516, -0.123626373626, -0.0103021978022, -0.288461538462 },
			{ -0.151098901099,   -0.0377747252747,    0.199175824176,  -0.0446428571429, 0.0755494505495, -0.00343406593407,   -0.123626373626, 0.0858516483516},
			{ -0.051510989011, 0.0274725274725, -0.0446428571429, 0.10989010989,   0.00343406593407, -0.0137362637363,  0.0927197802198, -0.123626373626},
			{ -0.123626373626,   -0.0858516483516, 0.0755494505495, 0.00343406593407,  0.199175824176,  0.0446428571429, -0.151098901099, 0.0377747252747},
			{ -0.0927197802198,  -0.123626373626, -0.00343406593407, -0.0137362637363,    0.0446428571429, 0.10989010989,   0.051510989011,  0.0274725274725},
			{ -0.0206043956044,  -0.0103021978022,  -0.123626373626, 0.0927197802198, -0.151098901099, 0.051510989011,  0.29532967033,   -0.133928571429},
			{ 0.0103021978022, -0.288461538462, 0.0858516483516, -0.123626373626, 0.0377747252747, 0.0274725274725, -0.133928571429, 0.384615384615 }});
			var areaOfElementComputed = element.AreaOfElement;
			var stiffnessOfElementComputed = element.StiffnessOfElement;
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
