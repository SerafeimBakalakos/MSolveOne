using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Entities;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.XFEM.IsoXFEM.IsoXfemElements;
using MGroup.XFEM.IsoXFEM.MeshGeneration;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.MSolve.Meshes.Structured;

namespace MGroup.XFEM.IsoXFEM.MeshGeneration
{
   public class MeshGeneration2D: IMeshGeneration
	{       
		public ElasticMaterial2D material;
		public GeometryProperties GeometryOfModel { get; }
		public MeshGeneration2D(ElasticMaterial2D material, GeometryProperties geometry)
        {            
            this.material = material;
            this.GeometryOfModel = geometry;
        }
		public Dictionary<int, IIsoXfemElement> CreateElements(Dictionary<int, XNode> nodes)
		{
			int el = 0;
			var elements = new Dictionary<int, IIsoXfemElement>();
			for (int i = 0; i < GeometryOfModel.NumberOfElementsX; i++)
			{
				for (int j = 0; j < GeometryOfModel.NumberOfElementsY; j++)
				{
					var node1ID = el + i;
					var node2ID = node1ID + GeometryOfModel.NumberOfElementsY + 1;
					var node3ID = node2ID + 1;
					var node4ID = node1ID + 1;
					var nodesOfElement = new[]
					{
							nodes[node1ID],
							nodes[node2ID],
							nodes[node3ID],
							nodes[node4ID]
					};
					var element = new IsoXfemElement2D(el, material, GeometryOfModel, nodesOfElement);
					element.SetIdOnAxis(new int[] { i, j });
					elements.Add(el,element);
					el = el + 1;
				}
			}
			return elements;
		}

		public Dictionary<int, XNode> CreateNodes()
		{
			var coordX = 0.0;
			var coordY = 0.0;
			int id = 0;
			var nodes = new Dictionary<int, XNode>();
			for (int i = 0; i < (GeometryOfModel.NumberOfElementsX + 1); i++)
			{
				for (int j = 0; j < (GeometryOfModel.NumberOfElementsY + 1); j++)
				{
					var nodeX = coordX;
					var nodeY = coordY;					
					double[] coords = { nodeX, nodeY };
					var node = new XNode(id, coords);				
					nodes.Add(id,node);
					coordY = coordY + GeometryOfModel.height / GeometryOfModel.NumberOfElementsY;
					id = id + 1;
				}
				coordY = 0;
				coordX = coordX + GeometryOfModel.length / GeometryOfModel.NumberOfElementsX;
			}
			return nodes;
		}

        public (Dictionary<int, XNode> nodes , Dictionary<int, IIsoXfemElement> elements) MakeMesh()
        {
            var nodes=CreateNodes();
            var elements=CreateElements(nodes);
			return (nodes,elements);			
        }

		public DualCartesianSimplicialSymmetricMesh2D CreateDualMesh()
		{
			int[] numElements = { GeometryOfModel.NumberOfElementsX, GeometryOfModel.NumberOfElementsY};
			int[] numNodesCoarse = { GeometryOfModel.NumberOfElementsX + 1, GeometryOfModel.NumberOfElementsY + 1 };
			int[] numNodesFine = { 2 * (numNodesCoarse[0] - 1) + 1, 2 * (numNodesCoarse[1] - 1) + 1 };
			double[] minCoords = { 0, 0};
			double[] maxCoords = { GeometryOfModel.length, GeometryOfModel.height};
			//var coarseMesh = new UniformCartesianMesh2D.Builder(minCoords, maxCoords, numElements)
			//	.SetMajorAxis(1) //TODO: Implement the other options in the mesh class and the builder.
			//							 //.SetElementNodeOrderDefault()
			//	.BuildMesh();
			//var fineMesh = new UniformSimplicialSymmetricMesh2D.Builder(minCoords, maxCoords, numNodesFine)
			//	.SetMajorAxis(1)
			//	.BuildMesh();
			var dualMesh = new DualCartesianSimplicialSymmetricMesh2D.Builder(
				minCoords, maxCoords, numNodesCoarse, numNodesFine)
				.SetMajorAxis(1).BuildMesh();
			return dualMesh;
		}
	}
}
