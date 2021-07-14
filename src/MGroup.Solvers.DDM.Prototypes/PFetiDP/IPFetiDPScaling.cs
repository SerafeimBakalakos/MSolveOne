using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	public interface IPFetiDPScaling
	{
		void Initialize();

		void InitializeComponents(PFetiDPSolver solver);
	}
}
