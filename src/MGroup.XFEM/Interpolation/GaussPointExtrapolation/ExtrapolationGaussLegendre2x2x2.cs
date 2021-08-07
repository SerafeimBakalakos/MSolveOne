using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Integration.Quadratures;

namespace MGroup.XFEM.Interpolation.GaussPointExtrapolation
{
	/// <summary>
	/// Calculates extrapolations of scalar , vector and tensor fields from the integration points of 2-by-2-by-2 Gauss-Legendre
	/// quadrature. this can be done at any point , but utility methods for directly outputting the extrapolated fields at the
	/// nodes of finite elements are also provided.
	/// </summary>
	public class ExtrapolationGaussLegendre2x2x2: GaussPointExtrapolationBase
	{
		private static readonly double sqrt3 = Math.Sqrt(3.0);
		private static readonly ExtrapolationGaussLegendre2x2x2 uniqueInstance = new ExtrapolationGaussLegendre2x2x2();

		private ExtrapolationGaussLegendre2x2x2() : base(GaussLegendre3D.GetQuadratureWithOrder(2, 2, 2)) { }

		/// <summary>
		/// Get the unique <see cref="ExtrapolationGaussLegendre2x2x2"/> object for the whole program. Thread safe.
		/// </summary>
		public static ExtrapolationGaussLegendre2x2x2 UniqueInstance => uniqueInstance;

		protected override double[] EvaluateExtrapolationFunctionsAt(double[] naturalPoint)
		{
			// Coordinates of the point in the auxiliary coordinate system of an imaginary "Gauss element" that has the Gauss 
			// points as its nodes.
			double r = sqrt3 * naturalPoint[0];
			double s = sqrt3 * naturalPoint[1];
			double t = sqrt3 * naturalPoint[2];

			// Shape functions of the imaginary "Gauss element". 
			// Each shape function corresponds to an integration point of Gauss-Legendre 2x2x2. Therefore their order is the same
			// as the one defined by GaussLegendre2D.Order2x2, namely Xi major/Eta minor, instead of the usual Quad4 order.
			var values = new double[8];                                 // The usual Hexa8 shape function would be:
			values[0] = 0.125 * (1 - r) * (1 - s) * (1 - t);	// N0
			values[1] = 0.125 * (1 + r) * (1 - s) * (1 - t);	// N1
			values[2] = 0.125 * (1 - r) * (1 + s) * (1 - t);	// N3
			values[3] = 0.125 * (1 + r) * (1 + s) * (1 - t);	// N2
			
			values[4] = 0.125 * (1 - r) * (1 - s) * (1 + t);	// N4
			values[5] = 0.125 * (1 + r) * (1 - s) * (1 + t);	// N5
			values[6] = 0.125 * (1 - r) * (1 + s) * (1 + t);	// N7
			values[7] = 0.125 * (1 + r) * (1 + s) * (1 + t);	// N6
			return values;
		}
	}
}
