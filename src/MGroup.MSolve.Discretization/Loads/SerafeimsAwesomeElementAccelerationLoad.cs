using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.MSolve.Discretization.Loads
{
	public class SerafeimsAwesomeElementAccelerationLoad : IElementLoad
	{
		public IElement Element { get; set; }

		public IList<MassAccelerationLoad> GlobalLoads { get; set; }

		public double[] CalcContribution()
		{
			return Element.ElementType.CalculateAccelerationForces(Element, GlobalLoads);
		}
	}
}
