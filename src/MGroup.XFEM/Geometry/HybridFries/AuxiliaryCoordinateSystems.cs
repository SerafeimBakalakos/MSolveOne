using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Commons;

namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 5.1
	/// </summary>
	public enum CrackedDomainRegion
	{
		Omega1 = 1, Omega2 = 2, Omega3 = 3, Omega4 = 4
	}

	public static class AuxiliaryCoordinateSystems
	{
		/// <summary>
		/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// section 5.2
		/// </summary>
		public static double CalcAlpha(double[] tripleLevelSetsOfPoint)
		{
			double phi2 = tripleLevelSetsOfPoint[1];
			double phi3 = tripleLevelSetsOfPoint[2];
			double alphaStar = Math.Sqrt(phi2 * phi2 - phi3 * phi3);
			CrackedDomainRegion region = DetermineRegion(tripleLevelSetsOfPoint);
			if (region == CrackedDomainRegion.Omega1)
			{
				return alphaStar;
			}
			else if (region == CrackedDomainRegion.Omega4)
			{
				return 0;
			}
			else // Omega2 || Omega3
			{
				return -alphaStar;
			}
		}

		/// <summary>
		/// WARNING: This is correct only if <paramref name="tripleLevelSetsOfPoint"/> are calculated from the explicit 
		/// description. Interpolating the level sets will cause gross errors.
		/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// section 5.1
		/// </summary>
		public static CrackedDomainRegion DetermineRegion(double[] tripleLevelSetsOfPoint, double tolerance = 1E-10)
		{
			//TODO: use an informed tolerance value, such as by taking into account max/min level sets
			var comparer = new ValueComparer(tolerance);

			double phi1 = tripleLevelSetsOfPoint[0];
			double phi2 = tripleLevelSetsOfPoint[1];
			double phi3 = tripleLevelSetsOfPoint[2];

			double phi3Abs = Math.Abs(phi3);
			if (!comparer.AreEqual(phi3Abs, phi1))
			{
				return CrackedDomainRegion.Omega1;
			}
			else if (!comparer.AreEqual(phi3Abs, phi2))
			{
				if (phi3 > 0)
				{
					return CrackedDomainRegion.Omega2;
				}
				else
				{
					return CrackedDomainRegion.Omega3;
				}
			}
			else
			{
				return CrackedDomainRegion.Omega4;
			}
		}
	}

	
}
