using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.Boundaries
{
	public class BoxDomainBoundary : IDomainBoundary
	{
		private readonly double[] minCoords;
		private readonly double[] maxCoords;

		public BoxDomainBoundary(double[] minCoords, double[] maxCoords)
		{
			this.minCoords = minCoords;
			this.maxCoords = maxCoords;
		}

		public bool SurroundsPoint(double[] point)
		{
			bool isInside = (point[0] > minCoords[0]) && (point[0] < maxCoords[0]);
			isInside &= (point[1] > minCoords[1]) && (point[1] < maxCoords[1]);
			isInside &= (point[2] > minCoords[2]) && (point[2] < maxCoords[2]);
			return isInside;
		}
	}
}
