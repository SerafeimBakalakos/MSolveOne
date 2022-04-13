namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System;
	using System.Collections.Generic;
	using System.Linq;
	using System.Text;

	using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.MSolve.Discretization.Loads;
	using MGroup.Solvers.Direct;
	using MGroup.Solvers.Iterative;
	using MGroup.XFEM.Entities;
	using MGroup.XFEM.IsoXFEM.IsoXfemElements;
	using MGroup.XFEM.IsoXFEM.MeshGeneration;
	using MGroup.XFEM.IsoXFEM.SolidRatioComputations;
	using MGroup.XFEM.Materials.Duplicates;

	using Xunit;

	public class Example3D
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
		//     .________________________________.
		//     |\                               |\
		//     | \                              | \
		//     |  \                             |  \
		//     |   \                            |   \#0
		//     |    . __________________________|____.
		//     |    |                           |\   |
		//     |    |                           | \  |
		//     |    |                           |  \ |
		//     |    |                           |   \|#1
		//     .____|___________________________.    |
		//      \   |                            \   |
		//       \  |                             \  |
		//        \ |                              \ |
		//         \|                               \|
		//          . _______________________________.#2
		public enum EdgeLoad
		{
			UpperEnd,
			MiddleEnd,
			BottomEnd
		}
		private static EdgeLoad endload;
		public static void RunExample3D()
		{			
			// Define material properties and geometry.			
			var geometry = new GeometryProperties(40, 20, 1, new int[] {40, 20, 1 });
			var material = new ElasticMaterial3D();
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			// Create mesh.
			var meshGeneration = new MeshGeneration3D(material, geometry);
			var (nodes, elements) = meshGeneration.MakeMesh();
			var dualMesh=meshGeneration.CreateDualMesh();
			// Add Constraints, Using enum Constrained Side.
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
			// X-Model creation.3-D Model.
			int dimension = 3;
			var xModel = new IsoXFEM.XModel<IIsoXfemElement>(dimension);
			xModel.Mesh = dualMesh;
			// Add Subdomain, Nodes and Elements to Model.
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
			// Add Loads. Using enum EndLoad in order to choose the node we want to apply the force.
			endload = EdgeLoad.MiddleEnd;
			//Z-Y-X
			int nodeIDLoad = (geometry.NumberOfElementsX + 1) * (geometry.NumberOfElementsY + 1) * (geometry.NumberOfElementsZ + 1) - (int)endload * (geometry.NumberOfElementsY / 2) * (geometry.NumberOfElementsZ + 1) - 1 - geometry.NumberOfElementsZ;
			//X-Y-Z
			//int nodeIDLoad = (geometry.NumberOfElementsX + 1) * (geometry.NumberOfElementsY + 1) * (geometry.NumberOfElementsZ + 1) - (int)endload * (geometry.NumberOfElementsY / 2) * (geometry.NumberOfElementsX + 1) - 1 - (geometry.NumberOfElementsZ)*(geometry.NumberOfElementsX+1)*(geometry.NumberOfElementsY+1);
			for (int i = 0; i < (geometry.NumberOfElementsZ + 1); i++)
			{
				Load load;
				load = new Load()
				{
					Node = xModel.Nodes[nodeIDLoad],
					DOF = StructuralDof.TranslationY,
					Amount = 0.5
				};
				xModel.NodalLoads.Add(load);
				nodeIDLoad++;
				//nodeIDLoad = nodeIDLoad + (geometry.NumberOfElementsX + 1) * (geometry.NumberOfElementsY + 1);
			}
			// Initialize the Model.
			xModel.Initialize();
			// Defines Skyline Solver.
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(xModel);
			var solver = solverFactory.BuildSolver(algebraicModel);
			#region Pcg Solver
			//var solverFactory = new PcgSolver.Factory();
			//var pcgBuilder = new PcgAlgorithm.Builder();
			//pcgBuilder.ResidualTolerance = 1E-12;
			//solverFactory.PcgAlgorithm = pcgBuilder.Build();
			//var algebraicModel = solverFactory.BuildAlgebraicModel(xModel);
			//var solver = solverFactory.BuildSolver(algebraicModel);
			#endregion
			// Defines solidRatio. The Problem is 3D so SolidVolume is selected.
			ISolidRatio solidRatio = new SolidVolume(xModel, Vector.CreateWithValue(xModel.Elements.Count, xModel.Elements.First().Value.SizeOfElement));
			// Defines the topology Optimization and Optimize the problem with IsoXfem Method.
			var topologyOptimization = new TopologyOptimization(xModel, solidRatio, solver, algebraicModel);
			topologyOptimization.IsoXfem();
		}
	}
}
