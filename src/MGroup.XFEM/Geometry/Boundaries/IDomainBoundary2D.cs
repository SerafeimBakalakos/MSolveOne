using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.Boundaries
{
	public interface IDomainBoundary2D
	{
		/// <summary>
		/// Returns true if <paramref name="point"/> is strictly inside the domain. Returns false if <paramref name="point"/>
		/// is outside the domain or on its boundary.
		/// </summary>
		/// <param name="point"></param>
		bool SurroundsPoint(double[] point);
	}
}
