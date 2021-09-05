using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

//TODO: Cache the coordinates and jacobians inside XPoint
namespace MGroup.XFEM.Enrichment.Functions
{
	public abstract class IsotropicBrittleTipEnrichments_v2 : ICrackTipEnrichment
	{
		private readonly ITipCoordinateSystem tipCoordSystem;

		protected IsotropicBrittleTipEnrichments_v2(ITipCoordinateSystem tipCoordSystem)
		{
			this.tipCoordSystem = tipCoordSystem;
		}

		public EvaluatedFunction EvaluateAllAt(XPoint point)
		{
			double[] polarCoords = tipCoordSystem.MapPointToLocalPolar(point);
			(double function, double[] gradientPolar) = EvaluateAllAt(polarCoords);
			double[] gradientGlobal = 
				tipCoordSystem.TransformScalarFieldDerivativesLocalPolarToGlobalCartesian(point, gradientPolar);
			return new EvaluatedFunction(function, gradientGlobal);
		}

		public double EvaluateAt(XNode node)
		{
			double[] polarCoords = tipCoordSystem.MapPointToLocalPolar(node);
			return EvaluateAt(polarCoords);
		}

		public double EvaluateAt(XPoint point)
		{
			double[] polarCoords = tipCoordSystem.MapPointToLocalPolar(point);
			return EvaluateAt(polarCoords);
		}

		public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point)
		{
			throw new NotImplementedException();
		}

		protected abstract (double function, double[] gradientPolar) EvaluateAllAt(double[] polarCoords);
		protected abstract double EvaluateAt(double[] polarCoords);

		public class Func0 : IsotropicBrittleTipEnrichments_v2
		{
			public Func0(ITipCoordinateSystem tipCoordSystem) : base(tipCoordSystem)
			{
			}

			protected override (double function, double[] gradientPolar) EvaluateAllAt(double[] polarCoords)
			{
				double sqrtR = Math.Sqrt(polarCoords[0]);
				double cosThetaHalf = Math.Cos(polarCoords[1] / 2.0);
				double sinThetaHalf = Math.Sin(polarCoords[1] / 2.0);

				double value = sqrtR * sinThetaHalf;
				var gradientPolar = new double[]
				{
					0.5 / sqrtR * sinThetaHalf, 0.5 * sqrtR * cosThetaHalf
				};

				return (value, gradientPolar);
			}

			protected override double EvaluateAt(double[] polarCoords)
			{
				return Math.Sqrt(polarCoords[0]) * Math.Sin(polarCoords[1] / 2.0);
			}
		}

		public class Func1 : IsotropicBrittleTipEnrichments_v2
		{
			public Func1(ITipCoordinateSystem tipCoordSystem) : base(tipCoordSystem)
			{
			}

			protected override (double function, double[] gradientPolar) EvaluateAllAt(double[] polarCoords)
			{
				double sqrtR = Math.Sqrt(polarCoords[0]);
				double cosThetaHalf = Math.Cos(polarCoords[1] / 2.0);
				double sinThetaHalf = Math.Sin(polarCoords[1] / 2.0);

				double value = sqrtR * cosThetaHalf;
				var gradientPolar = new double[]
				{
					0.5 / sqrtR * cosThetaHalf, -0.5 * sqrtR * sinThetaHalf
				};

				return (value, gradientPolar);
			}

			protected override double EvaluateAt(double[] polarCoords)
			{
				return Math.Sqrt(polarCoords[0]) * Math.Cos(polarCoords[1] / 2.0);
			}
		}

		public class Func2 : IsotropicBrittleTipEnrichments_v2
		{
			public Func2(ITipCoordinateSystem tipCoordSystem) : base(tipCoordSystem)
			{
			}

			protected override (double function, double[] gradientPolar) EvaluateAllAt(double[] polarCoords)
			{
				double sqrtR = Math.Sqrt(polarCoords[0]);
				double cosTheta = Math.Cos(polarCoords[1]);
				double sinTheta = Math.Sin(polarCoords[1]);
				double cosThetaHalf = Math.Cos(polarCoords[1] / 2.0);
				double sinThetaHalf = Math.Sin(polarCoords[1] / 2.0);

				double value = sqrtR * sinThetaHalf * sinTheta;
				var gradientPolar = new double[]
				{
					0.5 / sqrtR * sinThetaHalf * sinTheta, sqrtR * (0.5 * cosThetaHalf * sinTheta + sinThetaHalf * cosTheta)
				};

				return (value, gradientPolar);
			}

			protected override double EvaluateAt(double[] polarCoords)
			{
				return Math.Sqrt(polarCoords[0]) * Math.Sin(polarCoords[1] / 2.0) * Math.Sin(polarCoords[1]);
			}
		}

		public class Func3 : IsotropicBrittleTipEnrichments_v2
		{
			public Func3(ITipCoordinateSystem tipCoordSystem) : base(tipCoordSystem)
			{
			}

			protected override (double function, double[] gradientPolar) EvaluateAllAt(double[] polarCoords)
			{
				double sqrtR = Math.Sqrt(polarCoords[0]);
				double cosTheta = Math.Cos(polarCoords[1]);
				double sinTheta = Math.Sin(polarCoords[1]);
				double cosThetaHalf = Math.Cos(polarCoords[1] / 2.0);
				double sinThetaHalf = Math.Sin(polarCoords[1] / 2.0);

				double value = sqrtR * cosThetaHalf * sinTheta;
				var gradientPolar = new double[]
				{
					0.5 / sqrtR * cosThetaHalf * sinTheta, sqrtR * (-0.5 * sinThetaHalf * sinTheta + cosThetaHalf * cosTheta)
				};

				return (value, gradientPolar);
			}

			protected override double EvaluateAt(double[] polarCoords)
			{
				return Math.Sqrt(polarCoords[0]) * Math.Cos(polarCoords[1] / 2.0) * Math.Sin(polarCoords[1]);
			}
		}
	}
}
