using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DDM.FetiDP.Dofs
{
	public interface ICornerDofSelection
	{
		int[] CornerNodeIDs { get; }

		bool IsCornerDof(int nodeID, int dofID);
	}
}
