using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.MSolve.Discretization
{
	public interface IElementVectorProvider
	{
		double[] CalcVector(IElement element);
	}
}
