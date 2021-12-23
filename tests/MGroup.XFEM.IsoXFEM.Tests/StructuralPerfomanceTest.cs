namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.Materials.Duplicates;

	using Xunit;

	public class StructuralPerfomanceTest
	{
		[Fact]
		public void ComputeStrainEnergyandStrainEnergyDensity()
		{
			var geometry = new GeometryProperties(20, 20, 1, 2, 2);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var meshGeneration = new MeshGeneration(material, geometry);
			var mesh = meshGeneration.MakeMesh();
			int dimension = 2;
			var xModel = new XModel<IsoXfemElement2D>(dimension);
			foreach (var item in mesh.Item1.Keys)
			{
				xModel.Nodes[item] = mesh.Item1[item];
			}
			foreach (var item in mesh.Item2.Keys)
			{
				xModel.Elements[item] = mesh.Item2[item];
			}
			xModel.Initialize();
			double initialArea = 100;
			Vector displacements = Vector.CreateFromArray(new double[] { 0, 0, 0, 0, 0, 0, 1.76924266492499, 2.43273081966077, 0.174565886551294, 2.18549731446979, -2.05458622432392, 2.22010344064856, 2.24365209361063, 4.97664957363316, 0.162098204124805, 5.50209505074586, -3.20059751636863, 7.51106771335219 });
			var structuralPerfomance = new StructuralPerfomance(xModel.Nodes,xModel.Elements, initialArea);
			var (strainEnergyComputed,strainEnergyDensityComputed) = structuralPerfomance.ComputeStrainEnergyandStrainEnergyDensity(displacements);
			Vector strainEnergyExpected = Vector.CreateFromArray(new double[] { 1.08827975348119, 1.01527547939228, 0.331473388126520, 1.32050523567611 });
			Vector strainEnergyDensityExpected = Vector.CreateFromArray(new double[] { 0.0108827975348119, 0.0101527547939228, 0.00331473388126520, 0.0132050523567611 });
			for (int i = 0; i < strainEnergyExpected.Length; i++)
			{
				Assert.Equal(strainEnergyExpected[i], strainEnergyComputed[i], 8);
				Assert.Equal(strainEnergyDensityExpected[i], strainEnergyDensityComputed[i], 8);
			}
		}
		[Fact]
		public void ComputeNodalStrainEnergyDensityTest()
		{
			var geometry = new GeometryProperties(20, 20, 1, 2, 2);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var meshGeneration = new MeshGeneration(material, geometry);
			var mesh = meshGeneration.MakeMesh();
			int dimension = 2;
			var xModel = new XModel<IsoXfemElement2D>(dimension);
			foreach (var item in mesh.Item1.Keys)
			{
				xModel.Nodes[item] = mesh.Item1[item];
			}
			foreach (var item in mesh.Item2.Keys)
			{
				xModel.Elements[item] = mesh.Item2[item];
			}
			xModel.Initialize();
			double initialArea = 100;
			Vector displacements = Vector.CreateFromArray(new double[] { 0, 0, 0, 0, 0, 0, 1.76924266492499, 2.43273081966077, 0.174565886551294, 2.18549731446979, -2.05458622432392, 2.22010344064856, 2.24365209361063, 4.97664957363316, 0.162098204124805, 5.50209505074586, -3.20059751636863, 7.51106771335219 });
			var structuralPerfomance = new StructuralPerfomance(xModel.Nodes,xModel.Elements, initialArea);
			var (strainEnergyComputed,nodalStrainEnerdyDensityComputed) = structuralPerfomance.ComputeStrainEnergyAndNodalSEDensity(displacements);
			Vector strainEnergyDensity = Vector.CreateFromArray(new double[] { 0.0108827975348119, 0.0101527547939228, 0.00331473388126520, 0.0132050523567611 });
			Vector nosalStrainEnergyDensityExpected = Vector.CreateFromArray(new double[] { 0.0108827975348119, (0.0108827975348119 + 0.0101527547939228)/ 2, 0.0101527547939228,
			(0.0108827975348119+0.00331473388126520)/2, (0.0108827975348119+0.0101527547939228+0.00331473388126520+0.0132050523567611)/4,(0.0101527547939228+0.0132050523567611)/2,0.00331473388126520, (0.00331473388126520+0.0132050523567611)/2,0.0132050523567611});
			Assert.Equal(nosalStrainEnergyDensityExpected.Length, nodalStrainEnerdyDensityComputed.Length);
			for (int i = 0; i < nosalStrainEnergyDensityExpected.Length; i++)
			{
				Assert.Equal(nosalStrainEnergyDensityExpected[i], nodalStrainEnerdyDensityComputed[i], 10);
			}
		}
	}
}
