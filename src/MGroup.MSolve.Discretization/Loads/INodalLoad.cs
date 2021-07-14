using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.MSolve.Discretization.Loads
{
	public interface INodalLoad
	{
		IDofType DOF { get; }

		INode Node { get; }

		double Amount { get; }
	}
}
