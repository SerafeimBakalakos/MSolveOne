using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.MSolve.Discretization.Loads
{
	public interface INodalLoad
	{
		IDofType DOF { get; }

		INode Node { get; }

		double Amount { get; }
	}
}
