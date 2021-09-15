using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.Boundaries
{
	public interface IDomainBoundary3D
	{
		double[] CalcNormalAtBoundaryPoint(double[] point);

		RelativePositionManifoldPoint FindRelativePositionOf(double[] point);

		void MinimizeOffsetOfBoundaryPoint(double[] point);
	}
}
