using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.MSolve.Discretization.Loads
{
	public class ElementInternalRhsProvider : IElementVectorProvider
	{
		public double[] CalcVector(IElement element) => element.ElementType.CalculateForces(element);
	}
}
