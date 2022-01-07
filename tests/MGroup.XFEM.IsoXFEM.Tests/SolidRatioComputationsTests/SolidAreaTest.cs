namespace MGroup.XFEM.IsoXFEM.Tests.SolidRatioComputationsTests
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.XFEM.Entities;
	using MGroup.XFEM.IsoXFEM.SolidRatioComputations;
	using MGroup.XFEM.Materials.Duplicates;

	using Xunit;

	public class SolidAreaTest
	{
		//                      #2
		//     .________________________________.
		//     |                                |
		//     |                                |
		//     |                                |
		//     |                                |
		//     |                                |
		//  #3 |                                |      #1
		//     |                                |
		//     |                                |
		//     |                                |
		//     .________________________________.
		//                    #0
		public enum ConstrainedSide
		{
			Bottomside,
			Rightside,
			Upperside,
			Leftside,
		}
		private static ConstrainedSide constrainedSide = ConstrainedSide.Leftside;
		public enum EndLoad
		{
			UpperEnd,
			MiddleEnd,
			BottomEnd
		}
		private static EndLoad endload;
		[Fact]
		private void CalculateRatioTest()
		{
			var geometry = new GeometryProperties(2, 1, 1, 2, 1);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var meshGeneration = new MeshGeneration(material, geometry);
			var mesh = meshGeneration.MakeMesh();
			foreach (var item in mesh.Item1.Values)
			{
				switch (constrainedSide)
				{
					case ConstrainedSide.Bottomside:
						if (item.Y == 0)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Rightside:
						if (item.X == geometry.length)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Upperside:
						if (item.Y == geometry.height)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Leftside:
						if (item.X == 0)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					default:
						break;
				}
			}
			int dimension = 2;
			var xModel = new IsoXFEM.XModel<IsoXfemElement2D>(dimension);
			xModel.Subdomains[0] = new XSubdomain<IsoXfemElement2D>(0);
			foreach (var item in mesh.Item1.Keys)
			{
				xModel.Nodes[item] = mesh.Item1[item];
			}
			foreach (var item in mesh.Item2.Keys)
			{
				xModel.Elements[item] = mesh.Item2[item];
				xModel.Subdomains[0].Elements.Add(mesh.Item2[item]);
			}
			xModel.Initialize();
			Vector nodalStrainEnergyDensity = Vector.CreateFromArray(new double[] { -50, -50, 100, 100, -200, -200 });
			Vector initialAreas = Vector.CreateWithValue(2, 1);
			Vector areaOfElementsExpected = Vector.CreateFromArray(new double[] { 0.6666666666666666, 0.33333333333333 });
			var solidArea = new SolidArea(xModel, initialAreas);
			solidArea.RelativeCriteria = nodalStrainEnergyDensity;
			var areaOfElemenetsComputed = solidArea.CalculateSolidRatio();
			for (int i = 0; i < areaOfElementsExpected.Length; i++)
			{
				Assert.Equal(areaOfElementsExpected[i], areaOfElemenetsComputed[i], 10);
			}
		}
	}
}
