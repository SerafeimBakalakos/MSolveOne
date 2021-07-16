using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.DDM.LinearSystem
{
	public interface ISubdomainLinearSystem
	{
		ISubdomainFreeDofOrdering DofOrdering { get; }

		Vector RhsVector { get; }

		Vector Solution { get; set; }
	}
}
