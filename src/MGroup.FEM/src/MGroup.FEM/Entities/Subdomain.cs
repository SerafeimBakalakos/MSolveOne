using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.DataStructures;

//TODO: remove code that calculates rhs vector components (nodal loads, constraints, etc). It should be moved to dedicated 
//      classes like EquivalentLoadAssembler, so that it can be reused between subdomains of different projects (FEM, IGA, XFEM).
//TODO: same for multiscale
namespace MGroup.FEM.Entities
{
	public class Subdomain : ISubdomain
	{
		private readonly SortedDictionary<int, Node> nodes = new SortedDictionary<int, Node>();

		public Subdomain(int id)
		{
			this.ID = id;
		}

		public List<Element> Elements { get; } = new List<Element>();

		//public IList<EmbeddedNode> EmbeddedNodes { get; } = new List<EmbeddedNode>();

		public int ID { get; }

		//public bool MaterialsModified
		//{
		//    get
		//    {
		//        bool modified = false;
		//        foreach (Element element in elementsDictionary.Values)
		//            if (element.ElementType.MaterialModified)
		//            {
		//                modified = true;
		//                break;
		//            }
		//        return modified;
		//    }
		//}
		public bool LinearSystemModified { get; set; } = true; // At first it is modified

		//public void ClearMaterialStresses()
		//{
		//	foreach (Element element in Elements) element.ElementType.ClearMaterialStresses();
		//}

		public void DefineNodesFromElements()
		{
			nodes.Clear();
			foreach (Element element in Elements)
			{
				foreach (Node node in element.Nodes)
				{
					nodes[node.ID] = node;
				}
			}

			//foreach (var e in modelEmbeddedNodes.Where(x => nodeIDs.IndexOf(x.Node.ID) >= 0))
			//    EmbeddedNodes.Add(e);
		}

		public IEnumerable<IElement> EnumerateElements() => Elements;

		public IEnumerable<INode> EnumerateNodes() => nodes.Values;

		//TODO: This should be faster than looking up a SortedDictionary
		public int GetMultiplicityOfNode(int nodeID) => nodes[nodeID].Subdomains.Count;

		public void ResetMaterialsModifiedProperty()
		{
			this.LinearSystemModified = false;
			foreach (Element element in Elements) element.ElementType.ResetMaterialModified();
		}

		public void SaveMaterialState()
		{
			foreach (Element element in Elements) element.ElementType.SaveMaterialState();
		}
	}
}
