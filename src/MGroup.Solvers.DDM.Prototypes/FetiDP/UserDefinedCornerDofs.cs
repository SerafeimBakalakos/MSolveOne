using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DDM.Prototypes.FetiDP
{
	public class UserDefinedCornerDofSelection : ICornerDofSelection
	{
		private readonly HashSet<int> cornerNodes = new HashSet<int>();

		public UserDefinedCornerDofSelection()
		{
		}

		public int[] CornerNodeIDs => cornerNodes.ToArray();

		public void AddCornerNode(int nodeID) => cornerNodes.Add(nodeID);

		public bool IsCornerDof(INode node, IDofType type) => cornerNodes.Contains(node.ID);
	}
}
