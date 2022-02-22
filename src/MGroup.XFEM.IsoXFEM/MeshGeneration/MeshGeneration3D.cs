namespace MGroup.XFEM.IsoXFEM.MeshGeneration
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.MSolve.Meshes.Structured;
	using MGroup.XFEM.Entities;
	using MGroup.XFEM.Geometry.Mesh;
	using MGroup.XFEM.IsoXFEM.IsoXfemElements;
	using MGroup.XFEM.Materials.Duplicates;

	public class MeshGeneration3D : IMeshGeneration
	{
		public GeometryProperties GeometryOfModel { get; }
		public ElasticMaterial3D material;
		public MeshGeneration3D(ElasticMaterial3D material, GeometryProperties geometry)
		{
			this.material = material;
			this.GeometryOfModel = geometry;
		}
		#region Z-Y-X
		public Dictionary<int, IIsoXfemElement> CreateElements(Dictionary<int, XNode> nodes)
		{
			int el = 0;
			int idOfElement = 0;
			var elements = new Dictionary<int, IIsoXfemElement>();
			for (int i = 0; i < GeometryOfModel.NumberOfElementsX; i++)
			{
				el = i * el;
				for (int j = 0; j < GeometryOfModel.NumberOfElementsY; j++)
				{
					for (int k = 0; k < GeometryOfModel.NumberOfElementsZ; k++)
					{
						var node1ID = el + j;
						var node2ID = node1ID + (GeometryOfModel.NumberOfElementsY + 1) * (GeometryOfModel.NumberOfElementsZ + 1);
						var node3ID = node2ID + (GeometryOfModel.NumberOfElementsZ + 1);
						var node4ID = node1ID + (GeometryOfModel.NumberOfElementsZ + 1);
						var node5ID = node1ID + 1;
						var node6ID = node2ID + 1;
						var node7ID = node3ID + 1;
						var node8ID = node4ID + 1;
						var nodesOfElement = new[]
						{
							nodes[node1ID],
							nodes[node2ID],
							nodes[node3ID],
							nodes[node4ID],
							nodes[node5ID],
							nodes[node6ID],
							nodes[node7ID],
							nodes[node8ID]
						 };
						var element = new IsoXfemElement3D(idOfElement, material, GeometryOfModel, nodesOfElement);
						elements.Add(idOfElement, element);
						element.SetIdOnAxis(new int[] { i, j, k });
						el = el + 1;
						idOfElement++;
					}
				}
				el = (GeometryOfModel.NumberOfElementsY + 1) * (GeometryOfModel.NumberOfElementsZ + 1);
			}
			return elements;
		}

		public Dictionary<int, XNode> CreateNodes()
		{
			var coordX = 0.0;
			var coordY = 0.0;
			var coordZ = 0.0;
			int id = 0;
			var nodes = new Dictionary<int, XNode>();
			for (int i = 0; i < (GeometryOfModel.NumberOfElementsX + 1); i++)
			{
				for (int j = 0; j < (GeometryOfModel.NumberOfElementsY + 1); j++)
				{
					for (int k = 0; k < (GeometryOfModel.NumberOfElementsZ + 1); k++)
					{
						var nodeX = coordX;
						var nodeY = coordY;
						var nodeZ = coordZ;
						double[] coords = { nodeX, nodeY, nodeZ };
						var node = new XNode(id, coords);
						nodes.Add(id, node);
						coordZ = coordZ + GeometryOfModel.thickness / GeometryOfModel.NumberOfElementsZ;
						id = id + 1;
					}
					coordZ = 0;
					coordY = coordY + GeometryOfModel.height / GeometryOfModel.NumberOfElementsY;
				}
				coordZ = 0;
				coordY = 0;
				coordX = coordX + GeometryOfModel.length / GeometryOfModel.NumberOfElementsX;
			}
			return nodes;
		}
		#endregion
		//public Dictionary<int, IIsoXfemElement> CreateElements(Dictionary<int, XNode> nodes)
		//{
		//	int el = 0;
		//	int idOfElement = 0;
		//	var elements = new Dictionary<int, IIsoXfemElement>();
		//	for (int i = 0; i < GeometryOfModel.NumberOfElementsZ; i++)
		//	{
		//		el = i * el;
		//		for (int j = 0; j < GeometryOfModel.NumberOfElementsY; j++)
		//		{
		//			for (int k = 0; k < GeometryOfModel.NumberOfElementsX; k++)
		//			{
		//				var node1ID = el + j;
		//				var node2ID = node1ID + 1;
		//				var node3ID = node2ID + (GeometryOfModel.NumberOfElementsX + 1);
		//				var node4ID = node3ID - 1;
		//				var node5ID = node1ID + (GeometryOfModel.NumberOfElementsX+1)*(GeometryOfModel.NumberOfElementsY + 1);
		//				var node6ID = node5ID + 1;
		//				var node7ID = node6ID + (GeometryOfModel.NumberOfElementsX + 1);
		//				var node8ID = node7ID - 1;
		//				var nodesOfElement = new[]
		//				{
		//					nodes[node1ID],
		//					nodes[node2ID],
		//					nodes[node3ID],
		//					nodes[node4ID],
		//					nodes[node5ID],
		//					nodes[node6ID],
		//					nodes[node7ID],
		//					nodes[node8ID]
		//				 };
		//				var element = new IsoXfemElement3D(idOfElement, material, GeometryOfModel, nodesOfElement);
		//				elements.Add(idOfElement, element);
		//				element.SetIdOnAxis(new int[] { k, j, i });
		//				el = el + 1;
		//				idOfElement++;
		//			}
		//		}
		//		el = (GeometryOfModel.NumberOfElementsY + 1) * (GeometryOfModel.NumberOfElementsZ + 1);
		//	}
		//	return elements;
		//}

		//public Dictionary<int, XNode> CreateNodes()
		//{
		//	var coordX = 0.0;
		//	var coordY = 0.0;
		//	var coordZ = 0.0;
		//	int id = 0;
		//	var nodes = new Dictionary<int, XNode>();
		//	for (int i = 0; i < (GeometryOfModel.NumberOfElementsZ + 1); i++)
		//	{
		//		for (int j = 0; j < (GeometryOfModel.NumberOfElementsY + 1); j++)
		//		{
		//			for (int k = 0; k < (GeometryOfModel.NumberOfElementsX + 1); k++)
		//			{
		//				var nodeX = coordX;
		//				var nodeY = coordY;
		//				var nodeZ = coordZ;
		//				double[] coords = { nodeX, nodeY, nodeZ };
		//				var node = new XNode(id, coords);
		//				nodes.Add(id, node);
		//				coordX = coordX + GeometryOfModel.length / GeometryOfModel.NumberOfElementsX;
		//				id = id + 1;
		//			}
		//			coordX = 0;
		//			coordY = coordY + GeometryOfModel.height / GeometryOfModel.NumberOfElementsY;
		//		}
		//		coordX = 0;
		//		coordY = 0;
		//		coordZ = coordZ + GeometryOfModel.thickness / GeometryOfModel.NumberOfElementsZ;
		//	}
		//	return nodes;
		//}
		public (Dictionary<int, XNode> nodes, Dictionary<int, IIsoXfemElement> elements) MakeMesh()
		{
			var nodes = CreateNodes();
			var elements = CreateElements(nodes);
			return (nodes, elements);
		}
		public DualCartesianSimplicialMesh3D CreateDualMesh()
		{
			int[] numElements = { GeometryOfModel.NumberOfElementsX , GeometryOfModel.NumberOfElementsY ,GeometryOfModel.NumberOfElementsZ };
			int[] numNodes = { GeometryOfModel.NumberOfElementsX+1, GeometryOfModel.NumberOfElementsY+1, GeometryOfModel.NumberOfElementsZ +1};
			double[] minCoords = { 0, 0, 0 };
			double[] maxCoords = { GeometryOfModel.length, GeometryOfModel.height, GeometryOfModel.thickness };
			var coarseMesh = new UniformCartesianMesh3D.Builder(minCoords, maxCoords, numElements)
				.SetMajorMinorAxis(2, 0) //TODO: Implement the other options in the mesh class and the builder.
				.SetElementNodeOrderDefault()
				.BuildMesh();
			var fineMesh = new UniformSimplicialMesh3D.Builder(minCoords, maxCoords, numNodes)
				.SetMajorMinorAxis(2, 0)
				.BuildMesh();
			return new DualCartesianSimplicialMesh3D(coarseMesh, fineMesh);
		}
	}
}
