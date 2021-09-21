using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public interface ICrackTipSystem
	{
		/// <summary>
		/// Unit vector that points away from the crack body. It is orthogonal to <see cref="Normal"/>.
		/// </summary>
		double[] Extension { get; }

		/// <summary>
		/// Unit vector that is orthogonal to <see cref="Extension"/>. It does not always points towards the positive region
		/// of the signed distance function. E.g. for a tip at the start of a 2D crack, it points towards the negative half-plane.
		/// </summary>
		double[] Normal { get; }

		double[] TipCoordsGlobal { get; }

		double[] ExtendTowards(double angle, double length);

		//TODO: This probably applies to all tensors. However, the strain tensor used in MSolve has 2x the shear strains and may
		//		need some modification before passing it to this method.
		double[] RotateGlobalStressTensor(double[] globalStresses);
	}
}
