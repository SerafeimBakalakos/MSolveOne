using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Iterative.Preconditioning;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	public interface IPFetiDPPreconditioner : IPreconditioner
	{
		void Calculate();

		void InitializeComponents(PFetiDPSolver solver);
	}
}
