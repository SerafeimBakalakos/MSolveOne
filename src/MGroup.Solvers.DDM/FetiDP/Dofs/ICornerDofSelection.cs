using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DDM.FetiDP.Dofs
{
	public interface ICornerDofSelection
	{
		bool IsCornerDof(int subdomainID, int nodeID, int dofID);
	}
}
