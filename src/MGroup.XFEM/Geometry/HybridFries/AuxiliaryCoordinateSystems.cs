using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 5.1
	/// </summary>
	public enum CrackedDomainRegion
	{
		Omega1, Omega2, Omega3, Omega4
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
		/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// section 5.1. 
		/// </summary>
		/// <remarks>
		/// However, I prefer using the transformation in <see cref="MGroup.XFEM.Cracks.Geometry.TipCoordinateSystemImplicit"/> 
		/// since its jacobians are ill-conditioned only when r -> 0. Not to mention that its jacobians have been published in 
		/// at least 1 paper and can thus be verified easily.
		/// </remarks>
		public static double CalcTheta(double[] tripleLevelSetsOfPoint)
		{
			double phi2 = tripleLevelSetsOfPoint[1];
			double phi3 = tripleLevelSetsOfPoint[2];
			double thetaStar = Math.Asin(phi3 / phi2);
			CrackedDomainRegion region = DetermineRegion(tripleLevelSetsOfPoint);
			if (region == CrackedDomainRegion.Omega1)
			{
				return thetaStar;
			}
			else if (region == CrackedDomainRegion.Omega2)
			{
				return Math.PI - thetaStar;
			}
			else if (region == CrackedDomainRegion.Omega3)
			{
				return -Math.PI - thetaStar;
			}
			else
			{
				Debug.Assert((thetaStar == Math.PI / 2) || (thetaStar == -Math.PI / 2));
				return thetaStar;
			}
		}

		/// <summary>
		/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// section 5.1
		/// </summary>
		public static CrackedDomainRegion DetermineRegion(double[] tripleLevelSetsOfPoint)
		{
			double phi1 = tripleLevelSetsOfPoint[0];
			double phi2 = tripleLevelSetsOfPoint[1];
			double phi3 = tripleLevelSetsOfPoint[2];

			double phi3Abs = Math.Abs(phi3);
			if (phi1 != phi3Abs)
			{
				return CrackedDomainRegion.Omega1;
			}
			else if (phi2 != phi3Abs)
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
