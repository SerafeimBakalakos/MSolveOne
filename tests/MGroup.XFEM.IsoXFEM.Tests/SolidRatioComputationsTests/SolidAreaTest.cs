namespace MGroup.XFEM.IsoXFEM.Tests.SolidRatioComputationsTests
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.MSolve.Discretization.Loads;
	using MGroup.XFEM.Entities;
	using MGroup.XFEM.IsoXFEM.SolidRatioComputations;
	using MGroup.XFEM.Materials.Duplicates;
	using MGroup.XFEM.IsoXFEM.IsoXfemElements;

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
		//                      
		//     .________________________________.#0
		//     |                                |
		//     |                                |
		//     |                                |
		//     |                                |
		//     |                                |#1
		//     |                                |      
		//     |                                |
		//     |                                |
		//     |                                |
		//     .________________________________.#2
		//                    
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
			/// <summary>
			/// Define material properties and geometry.
			/// </summary>
			var geometry = new GeometryProperties(2, 1, 1, new int[] { 2,1 });
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			/// <summary>
			/// Create mesh.
			/// </summary>
			var meshGeneration = new MeshGeneration(material, geometry);
			var (nodes, elements) = meshGeneration.MakeMesh();
			/// <summary>
			/// Add Constraints, Using enum Constrained Side.
			/// </summary>
			constrainedSide = ConstrainedSide.Leftside;
			foreach (var node in nodes.Values)
			{
				switch (constrainedSide)
				{
					case ConstrainedSide.Bottomside:
						if (node.Y == 0)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Rightside:
						if (node.X == geometry.length)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Upperside:
						if (node.Y == geometry.height)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Leftside:
						if (node.X == 0)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					default:
						break;
				}
			}
			/// <summary>
			/// X-Model creation.2-D Model.
			/// </summary>
			int dimension = 2;
			var xModel = new IsoXFEM.XModel<IIsoXfemElement>(dimension);
			/// <summary>
			/// Add Subdomain, Nodes and Elements to Model.
			/// </summary>
			xModel.Subdomains[0] = new XSubdomain<IIsoXfemElement>(0);
			foreach (var node in nodes.Keys)
			{
				xModel.Nodes[node] = nodes[node];
			}
			foreach (var element in elements.Keys)
			{
				xModel.Elements[element] = elements[element];
				xModel.Subdomains[0].Elements.Add(elements[element]);
			}
			/// <summary>
			/// Add Loads. Using enum EndLoad in order to choose the node we want to apply the force.
			/// </summary>
			endload = EndLoad.BottomEnd;
			int nodeIDLoad = (geometry.NumberOfElementsX + 1) * (geometry.NumberOfElementsY + 1) - ((int)endload * (geometry.NumberOfElementsY) / 2) - 1;
			Load load;
			load = new Load()
			{
				Node = xModel.Nodes[nodeIDLoad],
				DOF = StructuralDof.TranslationY,
				Amount = 1
			};
			xModel.NodalLoads.Add(load);
			/// <summary>
			/// Initialize the Model.
			/// </summary>
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
