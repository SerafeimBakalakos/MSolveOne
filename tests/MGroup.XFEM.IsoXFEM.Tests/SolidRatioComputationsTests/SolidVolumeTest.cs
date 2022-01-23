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
	using MGroup.XFEM.IsoXFEM.IsoXfemElements;
	using MGroup.XFEM.IsoXFEM.MeshGeneration;
	using MGroup.XFEM.Materials.Duplicates;
	using MGroup.XFEM.IsoXFEM.SolidRatioComputations;

	using Xunit;

	public class SolidVolumeTest
	{
		//                      
		//     .________________________________.
		//     |\                               |\
		//     | \                              | \
		//     |  \                             |  \
		//     |   \                            |   \
		//     |    . __________________________|____.
		//     |    |                           |    |
		//     |    |                           |    |
		//     |    |                           |    |
		//     |    |                           |    |
		//     .____|___________________________.    |
		//      \   |                            \   |
		//       \  |                             \  |
		//        \ |                              \ |
		//         \|                               \|
		//          . _______________________________.
		public enum ConstrainedFace
		{
			Back,
			Front,
			Bottom,
			Up,
			Left,
			Right
		}
		private static ConstrainedFace constrainedFace = ConstrainedFace.Left;
		//                      
		//     .________________________________.#1
		//     |                                |
		//     |                                |
		//     |                                |
		//     |                                |
		//     |                                |#2
		//     |                                |      
		//     |                                |
		//     |                                |
		//     |                                |
		//     .________________________________.#3
		//                    
		public enum EdgeLoad
		{
			UpperEnd=1,
			MiddleEnd=2,
			BottomEnd=3
		}
		private static EdgeLoad endload;
		[Fact]
		private void CalculateRatioTest()
		{
			/// <summary>
			/// Define material properties and geometry.
			/// </summary>
			var geometry = new GeometryProperties(2, 2, 2, new int[] { 2, 2, 2 });
			var material = new ElasticMaterial3D();
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			/// <summary>
			/// Create mesh.
			/// </summary>
			IMeshGeneration meshGeneration = new MeshGeneration3D(material, geometry);
			var (nodes, elements) = meshGeneration.MakeMesh();
			/// <summary>
			/// Add Constraints, Using enum Constrained Side.
			/// </summary>
			constrainedFace = ConstrainedFace.Left;
			foreach (var node in nodes.Values)
			{
				switch (constrainedFace)
				{
					case ConstrainedFace.Back:
						if (node.Z == 0)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationZ, Amount = 0 });

						}
						break;
					case ConstrainedFace.Front:
						if (node.Z == geometry.thickness)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationZ, Amount = 0 });
						}
						break;
					case ConstrainedFace.Bottom:
						if (node.Y == 0)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationZ, Amount = 0 });
						}
						break;
					case ConstrainedFace.Up:
						if (node.Y == geometry.height)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationZ, Amount = 0 });
						}
						break;
					case ConstrainedFace.Left:
						if (node.X == 0)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationZ, Amount = 0 });
						}
						break;
					case ConstrainedFace.Right:
						if (node.X == geometry.length)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationZ, Amount = 0 });
						}
						break;
					default:
						break;
				}
			}
			/// <summary>
			/// X-Model creation.3-D Model.
			/// </summary>
			int dimension = 3;
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
			endload = EdgeLoad.MiddleEnd;
			int nodeIDLoad = (geometry.NumberOfElementsX + 1) * (geometry.NumberOfElementsY + 1)*(geometry.NumberOfElementsZ + 1) - (int)endload *(geometry.NumberOfElementsZ+1);
			for (int i = 0; i < (geometry.NumberOfElementsZ + 1); i++)
			{
				Load load;
				load = new Load()
				{
					Node = xModel.Nodes[nodeIDLoad],
					DOF = StructuralDof.TranslationY,
					Amount = 1
				};
				xModel.NodalLoads.Add(load);
				nodeIDLoad++;
			}
			/// <summary>
			/// Initialize the Model.
			/// </summary>
			xModel.Initialize();
			Vector nodalStrainEnergyDensity = Vector.CreateZero((geometry.NumberOfElementsX + 1) * (geometry.NumberOfElementsY + 1) * (geometry.NumberOfElementsZ + 1));
			for (int i = 0; i < 9; i++)
			{
				nodalStrainEnergyDensity[i] = -50;
				nodalStrainEnergyDensity[i + 9] = 100;
				nodalStrainEnergyDensity[i + 18] = -200;
			}
			Vector initialVolume = Vector.CreateWithValue(8, 1);
			Vector volumeOfElementsExpected = Vector.CreateFromArray(new double[] { 0.6666666666666666, 0.6666666666666666, 0.6666666666666666, 0.6666666666666666, 0.33333333333333, 0.33333333333333, 0.33333333333333, 0.33333333333333 });
			var solidVolume = new SolidVolume(xModel, initialVolume);
			solidVolume.RelativeCriteria = nodalStrainEnergyDensity;
			var volumeOfElemenetsComputed = solidVolume.CalculateSolidRatio();
			for (int i = 0; i < volumeOfElementsExpected.Length; i++)
			{
				Assert.Equal(volumeOfElementsExpected[i], volumeOfElemenetsComputed[i], 10);
			}
		}
	}
}

