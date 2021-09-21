using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public interface ICrackTipSystem
	{
		/// <summary>
		/// Unit vector. Orthogonal to <see cref="Normal"/>.
		/// </summary>
		double[] Extension { get; }

		/// <summary>
		/// Unit vector. Orthogonal to <see cref="Extension"/>.
		/// </summary>
		double[] Normal { get; }

		double[] TipCoordsGlobal { get; }

		double[] ExtendTowards(double angle, double length);
	}
}
