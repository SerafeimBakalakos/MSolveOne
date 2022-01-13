namespace MGroup.XFEM.IsoXFEM.Tests
{
	using Xunit;
	using System;
	using System.Collections.Generic;
	using System.Text;
	using System.Diagnostics;
	using MGroup.XFEM.IsoXFEM;
	using MGroup.XFEM.Entities;
	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.XFEM.Materials.Duplicates;

	public class MeshGenerationTests
	{
		[Fact]
		private void CreateNodesTest()
		{
			var geometry = new GeometryProperties(40, 40, 1, 2, 2);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var modelWithNodesExpected = new MeshGeneration(material, geometry);
			var nodesComputed = modelWithNodesExpected.CreateNodes();			
			var nodesExpected = new List<XNode>();
			nodesExpected.Add(new XNode(0, new double[] { 0, 0 } ));
			nodesExpected.Add(new XNode(1, new double[] { 0, 20 } ));
			nodesExpected.Add(new XNode(2, new double[] { 0, 40 } ));
			nodesExpected.Add(new XNode(3, new double[] { 20, 0 } ));
			nodesExpected.Add(new XNode(4, new double[] { 20, 20 }));
			nodesExpected.Add(new XNode(5, new double[] { 20, 40 }));
			nodesExpected.Add(new XNode(6, new double[] { 40, 0 }));
			nodesExpected.Add(new XNode(7, new double[] { 40, 20 }));
			nodesExpected.Add(new XNode(8, new double[] { 40, 40 }));
			for (int i = 0; i < nodesExpected.Count; i++)
			{
				Assert.Equal(nodesExpected[i].ID, nodesComputed[i].ID);
				Assert.Equal(nodesExpected[i].X, nodesComputed[i].X);
				Assert.Equal(nodesExpected[i].Y, nodesComputed[i].Y);
			}
		}
		[Fact]
		private void CreateElementTest()
		{
			var geometry = new GeometryProperties(40, 40, 1, 2, 2);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var model = new MeshGeneration(material, geometry);
			var nodes = model.CreateNodes();
			var elementsComputed = model.CreateElements(nodes);
			var elementsExpected = new List<IsoXfemElement2D>();
			elementsExpected.Add(new IsoXfemElement2D(0, material, geometry, new[]
			{
				nodes[0],
				nodes[3],
				nodes[4],
				nodes[1]
			}));
			elementsExpected.Add(new IsoXfemElement2D(1, material, geometry, new[]
			{
				nodes[1],
				nodes[4],
				nodes[5],
				nodes[2]
			}));
			elementsExpected.Add(new IsoXfemElement2D(2, material, geometry, new[]
			{
				nodes[3],
				nodes[6],
				nodes[7],
				nodes[4]
			}));
			elementsExpected.Add(new IsoXfemElement2D(3, material, geometry, new[]
			{
				nodes[4],
				nodes[7],
				nodes[8],
				nodes[5]
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
		//[Fact]
		//private void FindElementsOnNodesTests()
		//{
		//	var geometry = new GeometryProperties(40, 40, 1, 2, 2);
		//	var material = new ElasticMaterial2D(StressState2D.PlaneStress);
		//	material.YoungModulus = 1;
		//	material.PoissonRatio = 0.3;
		//	var model = new Model(material, geometry);
		//	model.CreateNodes();
		//	model.CreateElements();
		//	model.FindElementsOnNodes();
		//	var nodesWithElementsExpected = new List<List<IsoXfemElement2D>>();
		//	var list0 = new List<IsoXfemElement2D>();
		//	list0.Add(model.elements[0]);
		//	var list1 = new List<IsoXfemElement2D>();
		//	list1.Add(model.elements[0]);
		//	list1.Add(model.elements[1]);
		//	var list2 = new List<IsoXfemElement2D>();
		//	list2.Add(model.elements[1]);
		//	var list3 = new List<IsoXfemElement2D>();
		//	list3.Add(model.elements[0]);
		//	list3.Add(model.elements[2]);
		//	var list4= new List<IsoXfemElement2D>();
		//	list4.Add(model.elements[0]);
		//	list4.Add(model.elements[1]);
		//	list4.Add(model.elements[2]);
		//	list4.Add(model.elements[3]);
		//	var list5 = new List<IsoXfemElement2D>();
		//	list5.Add(model.elements[1]);
		//	list5.Add(model.elements[3]);
		//	var list6 = new List<IsoXfemElement2D>();
		//	list6.Add(model.elements[2]);
		//	var list7= new List<IsoXfemElement2D>();
		//	list7.Add(model.elements[2]);
		//	list7.Add(model.elements[3]);
		//	var list8 = new List<IsoXfemElement2D>();
		//	list8.Add(model.elements[3]);
		//	nodesWithElementsExpected.Add(list0);
		//	nodesWithElementsExpected.Add(list1);
		//	nodesWithElementsExpected.Add(list2);
		//	nodesWithElementsExpected.Add(list3);
		//	nodesWithElementsExpected.Add(list4);
		//	nodesWithElementsExpected.Add(list5);
		//	nodesWithElementsExpected.Add(list6);
		//	nodesWithElementsExpected.Add(list7);
		//	nodesWithElementsExpected.Add(list8);
		//	var nodesWithElementsComputed = model.nodes;
		//	for (int i = 0; i < nodesWithElementsComputed.Count; i++)
		//	{
		//		var nodeWithElementsComputed = nodesWithElementsComputed[i];
		//		var nodeWithElementsExpected = nodesWithElementsExpected[i];
		//		for (int j = 0; j < nodeWithElementsComputed.ElementsDictionary.Count; j++)
		//		{
		//			var key = nodeWithElementsExpected[j].ID;
		//			Assert.Equal(nodeWithElementsExpected[j].ID, nodeWithElementsComputed.ElementsDictionary[key].ID);
		//		}
		//	}
		//}
	}
}
