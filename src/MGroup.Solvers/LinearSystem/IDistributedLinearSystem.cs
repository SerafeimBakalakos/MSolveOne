using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Solvers.LinearSystem
{
	public interface IDistributedLinearSystem
	{
		DistributedVector RhsVector { get; }

		DistributedVector Solution { get; }
	}
}
