using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.MSolve.Discretization.Loads
{
	/// <summary>
	/// Applies a specific load along a freedom degree at all nodes.
	/// </summary>
	public interface IAllNodeLoad
	{
		IDofType DOF { get; }

		double Amount { get; }
	}
}
