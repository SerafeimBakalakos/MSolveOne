namespace MGroup.XFEM.IsoXFEM.Tests
{
	using Xunit;
	using System;
	using System.Collections.Generic;
	using System.Text;
	using System.Diagnostics;
	using MGroup.XFEM.IsoXFEM;
	public class ModelTests
	{
		[Fact]
		private void CreateNodesTest()
		{
			var geometry = new GeometryProperties(40, 40, 1, 2, 2);
			var material = new MaterialProperties(1, 0.3);
			var modelWithNodesExpected = new Model(material, geometry);
			modelWithNodesExpected.CreateNodes();
			var nodesComputed = modelWithNodesExpected.nodes;
			var nodesExpected = new List<Node>();
			nodesExpected.Add(new Node(0, 0, 0, true, true));
			nodesExpected.Add(new Node(1, 0, 20, true, true));
			nodesExpected.Add(new Node(2, 0, 40, true, true));
			nodesExpected.Add(new Node(3, 20, 0, false, false));
			nodesExpected.Add(new Node(4, 20, 20, false, false));
			nodesExpected.Add(new Node(5, 20, 40, false, false));
			nodesExpected.Add(new Node(6, 40, 0, false, false));
			nodesExpected.Add(new Node(7, 40, 20, false, false));
			nodesExpected.Add(new Node(8, 40, 40, false, false));
			for (int i = 0; i < nodesExpected.Count; i++)
			{
				Assert.Equal(nodesExpected[i].ID, nodesComputed[i].ID);
				Assert.Equal(nodesExpected[i].X, nodesComputed[i].X);
				Assert.Equal(nodesExpected[i].Y, nodesComputed[i].Y);
				Assert.Equal(nodesExpected[i].IsXConstrained, nodesComputed[i].IsXConstrained);
				Assert.Equal(nodesExpected[i].IsYConstrained, nodesComputed[i].IsYConstrained);
			}
		}
		[Fact]
		private void CreateElementTest()
		{
			var geometry = new GeometryProperties(40, 40, 1, 2, 2);
			var material = new MaterialProperties(1, 0.3);
			var model = new Model(material, geometry);
			model.CreateNodes();
			model.CreateElements();
			var elementsComputed = model.elements;
			var elementsExpected = new List<Element>();
			elementsExpected.Add(new Element(0, material, geometry, new[]
			{
				model.nodes[0],
				model.nodes[3],
				model.nodes[4],
				model.nodes[1]
			}));
			elementsExpected.Add(new Element(1, material, geometry, new[]
			{
				model.nodes[1],
				model.nodes[4],
				model.nodes[5],
				model.nodes[2]
			}));
			elementsExpected.Add(new Element(2, material, geometry, new[]
			{
				model.nodes[3],
				model.nodes[6],
				model.nodes[7],
				model.nodes[4]
			}));
			elementsExpected.Add(new Element(3, material, geometry, new[]
			{
				model.nodes[4],
				model.nodes[7],
				model.nodes[8],
				model.nodes[5]
			}));
			elementsExpected[0].dofsOfElement = new int[] { 0, 1, 6, 7, 8, 9, 2, 3 };
			elementsExpected[1].dofsOfElement = new int[] { 2, 3, 8, 9, 10, 11, 4, 5 };
			elementsExpected[2].dofsOfElement = new int[] { 6, 7, 12, 13, 14, 15, 8, 9 };
			elementsExpected[3].dofsOfElement = new int[] { 8, 9, 14, 15, 16, 17, 10, 11 };
			for (int i = 0; i < elementsComputed.Count; i++)
			{
				var elementComputed = elementsComputed[i];
				var elementExpected = elementsExpected[i];
				for (int j = 0; j < elementComputed.nodesOfElement.Count; j++)
				{
					Assert.Equal(elementExpected.nodesOfElement[j].ID, elementComputed.nodesOfElement[j].ID);
				}
				for (int j = 0; j < elementComputed.dofsOfElement.Length; j++)
				{
					Assert.Equal(elementExpected.dofsOfElement[j], elementComputed.dofsOfElement[j]);
				}
			}
		}
		[Fact]
		private void FindElementsOnNodesTests()
		{
			var geometry = new GeometryProperties(40, 40, 1, 2, 2);
			var material = new MaterialProperties(1, 0.3);
			var model = new Model(material, geometry);
			model.CreateNodes();
			model.CreateElements();
			model.FindElementsOnNodes();
			var nodesWithElementsExpected = new List<List<Element>>();
			var list0 = new List<Element>();
			list0.Add(model.elements[0]);
			var list1 = new List<Element>();
			list1.Add(model.elements[0]);
			list1.Add(model.elements[1]);
			var list2 = new List<Element>();
			list2.Add(model.elements[1]);
			var list3 = new List<Element>();
			list3.Add(model.elements[0]);
			list3.Add(model.elements[2]);
			var list4= new List<Element>();
			list4.Add(model.elements[0]);
			list4.Add(model.elements[1]);
			list4.Add(model.elements[2]);
			list4.Add(model.elements[3]);
			var list5 = new List<Element>();
			list5.Add(model.elements[1]);
			list5.Add(model.elements[3]);
			var list6 = new List<Element>();
			list6.Add(model.elements[2]);
			var list7= new List<Element>();
			list7.Add(model.elements[2]);
			list7.Add(model.elements[3]);
			var list8 = new List<Element>();
			list8.Add(model.elements[3]);
			nodesWithElementsExpected.Add(list0);
			nodesWithElementsExpected.Add(list1);
			nodesWithElementsExpected.Add(list2);
			nodesWithElementsExpected.Add(list3);
			nodesWithElementsExpected.Add(list4);
			nodesWithElementsExpected.Add(list5);
			nodesWithElementsExpected.Add(list6);
			nodesWithElementsExpected.Add(list7);
			nodesWithElementsExpected.Add(list8);
			var nodesWithElementsComputed = model.nodes;
			for (int i = 0; i < nodesWithElementsComputed.Count; i++)
			{
				var nodeWithElementsComputed = nodesWithElementsComputed[i];
				var nodeWithElementsExpected = nodesWithElementsExpected[i];
				for (int j = 0; j < nodeWithElementsComputed.elementsOnNode.Count; j++)
				{
					Assert.Equal(nodeWithElementsExpected[j].ID, nodeWithElementsComputed.elementsOnNode[j].ID);
				}
			}
		}
	}
}
