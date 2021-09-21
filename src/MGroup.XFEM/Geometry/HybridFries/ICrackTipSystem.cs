using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public interface ICrackTipSystem
	{
		double[] TipCoordsGlobal { get; }

		double[] ExtendTowards(double angle, double length);
	}
}
