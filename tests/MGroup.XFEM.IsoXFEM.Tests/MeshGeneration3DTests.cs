namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.XFEM.Entities;
	using MGroup.XFEM.IsoXFEM.IsoXfemElements;
	using MGroup.XFEM.IsoXFEM.MeshGeneration;
	using MGroup.XFEM.Materials.Duplicates;

	using Xunit;

	public class MeshGeneration3DTests
	{
		[Fact]
		private void CreateNodes3DTest()
		{
			var geometry = new GeometryProperties(20, 20, 20, new int[] { 2, 2, 2 });
			var material = new ElasticMaterial3D();
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			IMeshGeneration modelWithNodesExpected = new MeshGeneration3D(material, geometry);
			var nodesComputed = modelWithNodesExpected.CreateNodes();
			var nodesExpected = new List<XNode>();
			nodesExpected.Add(new XNode(0, new double[] { 0, 0, 0 }));
			nodesExpected.Add(new XNode(1, new double[] { 0, 0, 10 }));
			nodesExpected.Add(new XNode(2, new double[] { 0, 0, 20 }));
			nodesExpected.Add(new XNode(3, new double[] { 0, 10, 0 }));
			nodesExpected.Add(new XNode(4, new double[] { 0, 10, 10 }));
			nodesExpected.Add(new XNode(5, new double[] { 0, 10, 20 }));
			nodesExpected.Add(new XNode(6, new double[] { 0, 20, 0 }));
			nodesExpected.Add(new XNode(7, new double[] { 0, 20, 10 }));
			nodesExpected.Add(new XNode(8, new double[] { 0, 20, 20 }));
			nodesExpected.Add(new XNode(9, new double[] { 10, 0, 0 }));
			nodesExpected.Add(new XNode(10, new double[] { 10, 0, 10 }));
			nodesExpected.Add(new XNode(11, new double[] { 10, 0, 20 }));
			nodesExpected.Add(new XNode(12, new double[] { 10, 10, 0 }));
			nodesExpected.Add(new XNode(13, new double[] { 10, 10, 10 }));
			nodesExpected.Add(new XNode(14, new double[] { 10, 10, 20 }));
			nodesExpected.Add(new XNode(15, new double[] { 10, 20, 0 }));
			nodesExpected.Add(new XNode(16, new double[] { 10, 20, 10 }));
			nodesExpected.Add(new XNode(17, new double[] { 10, 20, 20 }));
			nodesExpected.Add(new XNode(18, new double[] { 20, 0, 0 }));
			nodesExpected.Add(new XNode(19, new double[] { 20, 0, 10 }));
			nodesExpected.Add(new XNode(20, new double[] { 20, 0, 20 }));
			nodesExpected.Add(new XNode(21, new double[] { 20, 10, 0 }));
			nodesExpected.Add(new XNode(22, new double[] { 20, 10, 10 }));
			nodesExpected.Add(new XNode(23, new double[] { 20, 10, 20 }));
			nodesExpected.Add(new XNode(24, new double[] { 20, 20, 0 }));
			nodesExpected.Add(new XNode(25, new double[] { 20, 20, 10 }));
			nodesExpected.Add(new XNode(26, new double[] { 20, 20, 20 }));
			for (int i = 0; i < nodesExpected.Count; i++)
			{
				Assert.Equal(nodesExpected[i].ID, nodesComputed[i].ID);
				Assert.Equal(nodesExpected[i].X, nodesComputed[i].X);
				Assert.Equal(nodesExpected[i].Y, nodesComputed[i].Y);
				Assert.Equal(nodesExpected[i].Z, nodesComputed[i].Z);
			}
		}
		[Fact]
		private void CreateElements3D()
		{
			var geometry = new GeometryProperties(20, 20, 20, new int[] { 2, 2, 2 });
			var material = new ElasticMaterial3D();
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			IMeshGeneration model = new MeshGeneration3D(material, geometry);
			var nodes = model.CreateNodes();
			var elementsComputed = model.CreateElements(nodes);
			var elementsExpected = new List<IsoXfemElement3D>();
			elementsExpected.Add(new IsoXfemElement3D(0, material, geometry, new[]
			{
				nodes[0],
				nodes[9],
				nodes[12],
				nodes[3],
				nodes[1],
				nodes[10],
				nodes[13],
				nodes[4]
			}));
			elementsExpected.Add(new IsoXfemElement3D(1, material, geometry, new[]
			{
				nodes[1],
				nodes[10],
				nodes[13],
				nodes[4],
				nodes[2],
				nodes[11],
				nodes[14],
				nodes[5]
			}));
			elementsExpected.Add(new IsoXfemElement3D(2, material, geometry, new[]
			{
				nodes[3],
				nodes[12],
				nodes[15],
				nodes[6],
				nodes[4],
				nodes[13],
				nodes[16],
				nodes[7]
			}));
			elementsExpected.Add(new IsoXfemElement3D(3, material, geometry, new[]
			{
				nodes[4],
				nodes[13],
				nodes[16],
				nodes[7],
				nodes[5],
				nodes[14],
				nodes[17],
				nodes[8]
			}));
			elementsExpected.Add(new IsoXfemElement3D(4, material, geometry, new[]
			{
				nodes[9],
				nodes[18],
				nodes[21],
				nodes[12],
				nodes[10],
				nodes[19],
				nodes[22],
				nodes[13]
			}));
			elementsExpected.Add(new IsoXfemElement3D(5, material, geometry, new[]
			{
				nodes[10],
				nodes[19],
				nodes[22],
				nodes[13],
				nodes[11],
				nodes[20],
				nodes[23],
				nodes[14]
			}));
			elementsExpected.Add(new IsoXfemElement3D(6, material, geometry, new[]
			{
				nodes[12],
				nodes[21],
				nodes[24],
				nodes[15],
				nodes[13],
				nodes[22],
				nodes[25],
				nodes[16]
			}));
			elementsExpected.Add(new IsoXfemElement3D(7, material, geometry, new[]
			{
				nodes[13],
				nodes[22],
				nodes[25],
				nodes[16],
				nodes[14],
				nodes[23],
				nodes[26],
				nodes[17]
			}));
			for (int i = 0; i < elementsComputed.Count; i++)
			{
				var elementComputed = elementsComputed[i];
				var elementExpected = elementsExpected[i];
				for (int j = 0; j < elementComputed.Nodes.Count; j++)
				{
					Assert.Equal(elementExpected.Nodes[j].ID, elementComputed.Nodes[j].ID);
				}
				for (int j = 0; j < elementComputed.Nodes.Count; j++)
				{
					Assert.Equal(elementExpected.Nodes[j], elementComputed.Nodes[j]);
				}
			}
		}
	}
}
