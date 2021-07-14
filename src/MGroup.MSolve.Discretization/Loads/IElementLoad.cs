using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;

namespace MGroup.MSolve.Discretization.Loads
{
	public interface IElementLoad
	{
		IElement Element { get; }

		double[] CalcContribution();
	}
}
