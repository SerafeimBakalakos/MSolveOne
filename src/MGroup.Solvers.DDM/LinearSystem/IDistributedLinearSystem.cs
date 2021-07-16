using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Solvers.DDM.LinearSystem
{
	public interface IDistributedLinearSystem
	{
		DistributedVector RhsVector { get; }

		DistributedVector Solution { get; }
	}
}
