using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Cracks.Geometry
{
	/// <summary>
	/// There are serious bugs in this. Theta is grossly incorrect in elements intersected by the crack.
	/// </summary>
	public class TipCoordinateSystemImplicitFries : ITipCoordinateSystem
	{
		private readonly IHybridFriesCrackDescription crack;

		public TipCoordinateSystemImplicitFries(IHybridFriesCrackDescription crack)
		{
			this.crack = crack;
		}

		public double[] MapPointToLocalPolar(XNode node)
		{
			double[] levelSets = crack.GetTripleLevelSetsOf(node);
			return CalcPolarCoords(levelSets);
		}

		public double[] MapPointToLocalPolar(XPoint point)
		{
			//TODO: Cache the coordinates inside XPoint. Even better, the tip functions should do that!!!
			double[] levelSets = crack.InterpolateTripleLevelSets(point);
			return CalcPolarCoords(levelSets);
		}

		public double[] TransformScalarFieldDerivativesLocalPolarToGlobalCartesian(XPoint point, double[] gradientPolar)
		{
			//TODO: Cache the jacobian polar -> cartesian inside XPoint. Even better, the tip functions should do that!!!
			Matrix jacobianNatural2LevelSet = CalcJacobianNatural2LevelSet(point);
			Matrix jacobianLevelSet2Polar = CalcJacobianLevelSet2Polar(point);
			Matrix jacobianGlobal2Polar = 
				jacobianLevelSet2Polar * jacobianNatural2LevelSet * point.JacobianNaturalGlobal.InverseMatrix;
			return jacobianGlobal2Polar.Multiply(gradientPolar);
		}

		/// <remarks>
		/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// section 5.1. 
		/// </remarks>
		/// <param name="tripleLevelSetsOfPoint"></param>
		/// <returns></returns>
		private double[] CalcPolarCoords(double[] tripleLevelSetsOfPoint)
		{
			double phi2 = tripleLevelSetsOfPoint[1];
			double phi3 = tripleLevelSetsOfPoint[2];
			double thetaStar = Math.Asin(phi3 / phi2);
			CrackedDomainRegion region = AuxiliaryCoordinateSystems.DetermineRegion(tripleLevelSetsOfPoint);
			double theta;
			if (region == CrackedDomainRegion.Omega1)
			{
				theta = thetaStar;
			}
			else if (region == CrackedDomainRegion.Omega2)
			{
				theta = Math.PI - thetaStar;
			}
			else if (region == CrackedDomainRegion.Omega3)
			{
				theta = -Math.PI - thetaStar;
			}
			else
			{
				Debug.Assert((thetaStar == Math.PI / 2) || (thetaStar == -Math.PI / 2));
				theta = thetaStar;
			}

			return new double[] { phi2, theta };
		}


		/// <summary>
		/// Both 2D and 3D: J = [ r,phi1 r,phi2 r,phi3 ; theta,phi1 theta,phi2 theta,phi3 ]
		/// </summary>
		/// <remarks>
		/// Based on "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// section 5.1. 
		/// </remarks>
		/// <param name="point"></param>
		private Matrix CalcJacobianLevelSet2Polar(XPoint point)
		{
			//TODO: the level sets and polar coordinates should be injected by the caller, perhaps stores as coordinates of XPoint
			double[] levelSets = crack.InterpolateTripleLevelSets(point);
			CrackedDomainRegion region = AuxiliaryCoordinateSystems.DetermineRegion(levelSets);

			//double phi1 = levelSets[0];
			double phi2 = levelSets[1];
			double phi3 = levelSets[2];

			// Derivatives of r
			var result = Matrix.CreateZero(2, 3);
			result[0, 0] = 0;
			result[0, 1] = 1;
			result[0, 2] = 0;

			// Derivatives of theta
			double alphaStar = Math.Sqrt(phi2 * phi2 - phi3 * phi3);
			double thetaStar_phi2 = -(phi3 / phi2) / alphaStar; // d(thetaStar)/d(phi2) 
			double thetaStar_phi3 = phi2 / alphaStar;
			//double thetaStar_phi1 = 0;
			if (region == CrackedDomainRegion.Omega1)
			{
				result[1, 0] = 0;
				result[1, 1] = thetaStar_phi2;
				result[1, 2] = thetaStar_phi3;
			}
			else if ((region == CrackedDomainRegion.Omega2) || (region == CrackedDomainRegion.Omega3))
			{
				result[1, 0] = 0;
				result[1, 1] = -thetaStar_phi2;
				result[1, 2] = -thetaStar_phi3;
			}
			else
			{
				result[1, 0] = 0;
				result[1, 1] = 0;
				result[1, 2] = 0;
			}

			return result;
		}

		/// <summary>
		/// 2D: J = [ phi1,xi phi1,eta ; phi2,xi phi2,eta ; phi3,xi phi3,zeta ].
		/// 3D: J = [ phi1,xi phi1,eta phi1,zeta ; phi2,xi phi2,eta phi2,zeta ; phi3,xi phi3,eta phi3,zeta ].
		/// E.g. phi1,xi = sum(N_i,xi * phi1_i), where i are the nodes of the element that contains <paramref name="point"/>.
		/// </summary>
		/// <remarks>
		/// Based on "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// section 4.3. 
		/// </remarks>
		/// <param name="point"></param>
		private Matrix CalcJacobianNatural2LevelSet(XPoint point)
		{
			int dim = point.Dimension;
			var result = Matrix.CreateZero(3, dim);
			IReadOnlyList<XNode> nodes = point.Element.Nodes;
			for (int n = 0; n < nodes.Count; ++n)
			{
				double[] levelSets = crack.GetTripleLevelSetsOf(nodes[n]);
				for (int d = 0; d < dim; ++d)
				{
					double dN = point.ShapeFunctionDerivatives[n, d];
					result[0, d] += dN * levelSets[0]; // phi1 derivative
					result[1, d] += dN * levelSets[1]; // phi2 derivative
					result[2, d] += dN * levelSets[2]; // phi3 derivative
				}
			}
			return result;
		}
	}
}
