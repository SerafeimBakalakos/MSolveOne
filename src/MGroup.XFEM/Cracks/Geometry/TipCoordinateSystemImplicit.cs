using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Cracks.Geometry
{
	public class TipCoordinateSystemImplicit : ITipCoordinateSystem
	{
		private readonly IImplicitCrackGeometry crack;

		public TipCoordinateSystemImplicit(IImplicitCrackGeometry crack)
		{
			this.crack = crack;
		}

		public double[] MapPointToLocalPolar(XNode node)
		{
			double[] levelSets = crack.GetNodalLevelSets(node);
			return CalcPolarCoords(levelSets);
		}

		public double[] MapPointToLocalPolar(XPoint point)
		{
			//TODO: Cache the coordinates inside XPoint. Even better, the tip functions should do that!!!
			double[] levelSets = crack.InterpolateLevelSets(point);
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
		/// See Shi et al. "Abaqus Implementation of Extended Finite Element Method Using a Level Set Representation for 
		/// Three-Dimensional Fatigue Crack Growth and Life Predictions" (2010), text after Eq 3. 
		/// </remarks>
		/// <param name="levelSets"></param>
		/// <returns></returns>
		private double[] CalcPolarCoords(double[] levelSets)
		{
			double phi = levelSets[0];
			double psi = levelSets[1];
			double r = Math.Sqrt(phi * phi + psi * psi);
			double theta = Math.Atan2(phi, psi);
			return new double[] { r, theta };
		}

		/// <summary>
		/// Both 2D and 3D: J = [ r,phi r,psi ; theta,phi theta,psi ]
		/// </summary>
		/// <remarks>
		/// See Shi et al. "Abaqus Implementation of Extended Finite Element Method Using a Level Set Representation for 
		/// Three-Dimensional Fatigue Crack Growth and Life Predictions" (2010), Eq 10. 
		/// </remarks>
		/// <param name="point"></param>
		private Matrix CalcJacobianLevelSet2Polar(XPoint point)
		{
			//TODO: the level sets and polar coordinates should be injected by the caller, perhaps stores as coordinates of XPoint
			double[] levelSets = crack.InterpolateLevelSets(point);
			double[] polarCoords = CalcPolarCoords(levelSets);

			double phi = levelSets[0];
			double psi = levelSets[1];
			double r = polarCoords[0];
			double r2 = r * r;

			var result = Matrix.CreateZero(2, 2);
			result[0, 0] = phi / r;
			result[0, 1] = psi / r;
			result[1, 0] = psi / r2;
			result[1, 1] = - phi / r2;

			return result;
		}

		/// <summary>
		/// 2D: J = [ phi,xi phi,eta; psi,xi psi,eta ].
		/// 3D: J = [ phi,xi phi,eta phi,zeta; psi,xi psi,eta psi,zeta ].
		/// E.g. phi,xi = sum(N_i,xi * phi_i), where i are the nodes of the element that contains <paramref name="point"/>.
		/// </summary>
		/// <remarks>
		/// See Shi et al. "Abaqus Implementation of Extended Finite Element Method Using a Level Set Representation for 
		/// Three-Dimensional Fatigue Crack Growth and Life Predictions" (2010), Eq 11. 
		/// </remarks>
		/// <param name="point"></param>
		private Matrix CalcJacobianNatural2LevelSet(XPoint point)
		{
			int dim = point.Dimension;
			var result = Matrix.CreateZero(2, dim);
			IReadOnlyList<XNode> nodes = point.Element.Nodes;
			for (int n = 0; n < nodes.Count; ++n)
			{
				double[] levelSets = crack.GetNodalLevelSets(nodes[n]);
				for (int d = 0; d < dim; ++d)
				{
					double dN = point.ShapeFunctionDerivatives[d, n];
					result[0, d] += dN * levelSets[0]; // phi derivative
					result[1, d] += dN * levelSets[1]; // psi derivative
				}
			}
			return result;
		}
	}
}
