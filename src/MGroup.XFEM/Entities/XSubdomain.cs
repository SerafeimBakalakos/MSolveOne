using System;
using System.Collections.Generic;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Elements;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.XFEM.Entities
{
	public class XSubdomain<TElement> : ISubdomain
		where TElement : class, IXFiniteElement 
	{
		private readonly SortedDictionary<int, XNode> nodes = new SortedDictionary<int, XNode>();

		public XSubdomain(int id)
		{
			this.ID = id;
		}

		public Table<INode, IDofType, double> Constraints { get; } = new Table<INode, IDofType, double>();

		public List<TElement> Elements { get; } = new List<TElement>();

		public Vector Forces { get; set; } //TODO: this doesn't belong here

		public int ID { get; }

		public bool LinearSystemModified { get; set; } = true;

		public void ClearMaterialStresses() => throw new NotImplementedException();

		public void DefineNodesFromElements()
		{
			nodes.Clear();
			foreach (TElement element in Elements)
			{
				foreach (XNode node in element.Nodes)
				{
					nodes[node.ID] = node;
				}
			}
		}

		IEnumerable<IElement> ISubdomain.EnumerateElements() => Elements;

		public IEnumerable<TElement> EnumerateElements() => Elements;

		public IEnumerable<INode> EnumerateNodes() => nodes.Values;

		public int GetMultiplicityOfNode(int nodeID) => nodes[nodeID].Subdomains.Count;

		public void ResetMaterialsModifiedProperty() => throw new NotImplementedException();

		public void SaveMaterialState() => throw new NotImplementedException();
	}
}
