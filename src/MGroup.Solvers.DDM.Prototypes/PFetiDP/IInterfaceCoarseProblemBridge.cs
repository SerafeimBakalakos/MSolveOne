using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	public interface IInterfaceCoarseProblemBridge
	{
		void InitializeComponents(PFetiDPSolver solver);

		void LinkInterfaceCoarseProblems();
	}
}
