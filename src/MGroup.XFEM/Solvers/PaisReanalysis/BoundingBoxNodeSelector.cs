using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.Extensions;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class BoundingBoxNodeSelector : IEnrichedNodeSelector
	{
		private readonly int dim;
		private readonly double[] minCoords;
		private readonly double[] maxCoords;

		public BoundingBoxNodeSelector(double[] minCoords, double[] maxCoords)
		{
			this.dim = minCoords.Length;
			this.minCoords = minCoords;
			this.maxCoords = maxCoords;
		}

		public bool CanNodeBeEnriched(INode node)
		{
			double[] x = node.Coordinates();
			for (int d = 0; d < dim; ++d)
			{
				if ((x[d] < minCoords[d]) || (x[d] > maxCoords[d]))
				{
					return false;
				}
			}
			return true;
		}
	}
}
