using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Solvers.DDM.Prototypes.FetiDP
{
	public interface IFetiDPCoarseProblem
	{
		void FindDofs();

		void Prepare();
	}
}
