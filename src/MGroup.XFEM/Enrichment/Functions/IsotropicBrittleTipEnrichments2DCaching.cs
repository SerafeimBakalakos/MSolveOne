using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

//TODO: I could make an IXOpenGeometry:IXGeometryDescription that also exposes tip data and use that here instead of the delegate.
//      However, how would these classes choose which tip to use, if there are multiple ones? With the delegate this is very easy.
//TODO: Make this thread-safe
namespace MGroup.XFEM.Enrichment.Functions
{
	/// <summary>
	/// This implementation only does the geometric computations once for all 4 tip functions. However by caching the common
	/// intermediate results, it is no longer thread-safe. In fact it has been shown to cause race conditions.
	/// </summary>
	public class IsotropicBrittleTipEnrichments2DCaching
	{
		private readonly Func<TipCoordinateSystemExplicit> getTipSystem;

		private NodePolarDataCache nodeCache;
		private PointPolarDataCache pointCache;

		public IsotropicBrittleTipEnrichments2DCaching(Func<TipCoordinateSystemExplicit> getTipSystem)
		{
			this.getTipSystem = getTipSystem;

			Functions = new ICrackTipEnrichment[4];
			Functions[0] = new Func0(this);
			Functions[1] = new Func1(this);
			Functions[2] = new Func2(this);
			Functions[3] = new Func3(this);
		}

		public ICrackTipEnrichment[] Functions { get; }

		//TODO: Perhaps these methods should be in a separate class, not the one that contains the function classes.
		private (double[] coords, TipJacobiansExplicit jacobians) CalcAll(XPoint point)
		{
			if ((pointCache == null) || (pointCache.originalPoint != point))
			{
				TipCoordinateSystemExplicit tipSystem = getTipSystem();
				double[] cartesianCoords = GetGlobalCartesianCoords(point);
				var polarCoords = tipSystem.MapPointGlobalCartesianToLocalPolar(cartesianCoords);
				TipJacobiansExplicit jacobians = tipSystem.CalcJacobiansAt(polarCoords);
				pointCache = new PointPolarDataCache(point, polarCoords, jacobians);
			}
			return (pointCache.polarCoords, pointCache.polarJacobians);
		}

		private double[] CalcPolarCoordinates(XNode node)
		{
			if ((nodeCache == null) || (nodeCache.originalNode != node))
			{
				TipCoordinateSystemExplicit tipSystem = getTipSystem();
				var polarCoords = tipSystem.MapPointGlobalCartesianToLocalPolar(node.Coordinates);
				nodeCache = new NodePolarDataCache(node, polarCoords);
			}
			return nodeCache.polarCoords;
		}

		private double[] CalcPolarCoordinates(XPoint point)
		{
			if ((pointCache == null) || (pointCache.originalPoint != point))
			{
				TipCoordinateSystemExplicit tipSystem = getTipSystem();
				double[] cartesianCoords = GetGlobalCartesianCoords(point);
				var polarCoords = tipSystem.MapPointGlobalCartesianToLocalPolar(cartesianCoords);
				pointCache = new PointPolarDataCache(point, polarCoords, null);
			}
			return pointCache.polarCoords;
		}

		private double[] GetGlobalCartesianCoords(XPoint point)
		{
			bool hasCartesian = point.Coordinates.TryGetValue(CoordinateSystem.GlobalCartesian, out double[] cartesianCoords);
			if (!hasCartesian)
			{
				cartesianCoords = point.MapCoordinates(point.ShapeFunctions, point.Element.Nodes);
				point.Coordinates[CoordinateSystem.GlobalCartesian] = cartesianCoords;
			}
			return cartesianCoords;
		}

		public class Func0 : ICrackTipEnrichment
		{
			private readonly IsotropicBrittleTipEnrichments2DCaching commonData;

			public Func0(IsotropicBrittleTipEnrichments2DCaching commonData)
			{
				this.commonData = commonData;
			}

			public EvaluatedFunction EvaluateAllAt(XPoint point)
			{
				(double[] polarCoords, TipJacobiansExplicit jacobians) = commonData.CalcAll(point);
				double sqrtR = Math.Sqrt(polarCoords[0]);
				double cosThetaHalf = Math.Cos(polarCoords[1] / 2.0);
				double sinThetaHalf = Math.Sin(polarCoords[1] / 2.0);

				double value = sqrtR * sinThetaHalf;
				var gradientPolar = Vector.CreateFromArray(new double[]
				{
					0.5 / sqrtR * sinThetaHalf, 0.5 * sqrtR * cosThetaHalf
				});

				Vector gradientGlobal = jacobians.TransformScalarFieldDerivativesLocalPolarToGlobalCartesian(gradientPolar);
				return new EvaluatedFunction(value, gradientGlobal.RawData);
			}

			public double EvaluateAt(XNode node)
			{
				double[] polarCoords = commonData.CalcPolarCoordinates(node);
				return Math.Sqrt(polarCoords[0]) * Math.Sin(polarCoords[1] / 2.0);
			}

			public double EvaluateAt(XPoint point)
			{
				double[] polarCoords = commonData.CalcPolarCoordinates(point);
				return Math.Sqrt(polarCoords[0]) * Math.Sin(polarCoords[1] / 2.0);
			}

			public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point)
			{
				throw new NotImplementedException();
				double[] polarCoords = commonData.CalcPolarCoordinates(point);
				Debug.Assert(polarCoords[1] == Math.PI/2); //TODO: this needs some tolerance. Also why would it not be -pi/2?
				double sqrtR = Math.Sqrt(polarCoords[0]);
				double jumpAbsolute = sqrtR * 2; // sqrt(r) * (sin(pi/2) - sin(-pi/2))
				double sign = +1; //TODO: The positive side of sin(theta/2) coincides with the positive side of crack, only if there is a single tip. How can I check this?
				return sign * jumpAbsolute;
			}
		}

		public class Func1 : ICrackTipEnrichment
		{
			private readonly IsotropicBrittleTipEnrichments2DCaching commonData;

			public Func1(IsotropicBrittleTipEnrichments2DCaching commonData)
			{
				this.commonData = commonData;
			}

			public EvaluatedFunction EvaluateAllAt(XPoint point)
			{
				(double[] polarCoords, TipJacobiansExplicit jacobians) = commonData.CalcAll(point);
				double sqrtR = Math.Sqrt(polarCoords[0]);
				double cosThetaHalf = Math.Cos(polarCoords[1] / 2.0);
				double sinThetaHalf = Math.Sin(polarCoords[1] / 2.0);

				double value = sqrtR * cosThetaHalf;
				var gradientPolar = Vector.CreateFromArray(new double[]
				{
					0.5 / sqrtR * cosThetaHalf, -0.5 * sqrtR * sinThetaHalf
				});

				Vector gradientGlobal = jacobians.TransformScalarFieldDerivativesLocalPolarToGlobalCartesian(gradientPolar);
				return new EvaluatedFunction(value, gradientGlobal.RawData);
			}

			public double EvaluateAt(XNode node)
			{
				double[] polarCoords = commonData.CalcPolarCoordinates(node);
				return Math.Sqrt(polarCoords[0]) * Math.Cos(polarCoords[1] / 2.0);
			}

			public double EvaluateAt(XPoint point)
			{
				double[] polarCoords = commonData.CalcPolarCoordinates(point);
				return Math.Sqrt(polarCoords[0]) * Math.Cos(polarCoords[1] / 2.0);
			}

			public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point) => 0;
		}

		public class Func2 : ICrackTipEnrichment
		{
			private readonly IsotropicBrittleTipEnrichments2DCaching commonData;

			public Func2(IsotropicBrittleTipEnrichments2DCaching commonData)
			{
				this.commonData = commonData;
			}

			public EvaluatedFunction EvaluateAllAt(XPoint point)
			{
				(double[] polarCoords, TipJacobiansExplicit jacobians) = commonData.CalcAll(point); 
				double sqrtR = Math.Sqrt(polarCoords[0]);
				double cosTheta = Math.Cos(polarCoords[1]);
				double sinTheta = Math.Sin(polarCoords[1]);
				double cosThetaHalf = Math.Cos(polarCoords[1] / 2.0);
				double sinThetaHalf = Math.Sin(polarCoords[1] / 2.0);

				double value = sqrtR * sinThetaHalf * sinTheta;
				var gradientPolar = Vector.CreateFromArray(new double[]
				{
					0.5 / sqrtR * sinThetaHalf * sinTheta, sqrtR * (0.5 * cosThetaHalf * sinTheta + sinThetaHalf * cosTheta)
				});

				Vector gradientGlobal = jacobians.TransformScalarFieldDerivativesLocalPolarToGlobalCartesian(gradientPolar);
				return new EvaluatedFunction(value, gradientGlobal.RawData);
			}

			public double EvaluateAt(XNode node)
			{
				double[] polarCoords = commonData.CalcPolarCoordinates(node);
				return Math.Sqrt(polarCoords[0]) * Math.Sin(polarCoords[1] / 2.0) * Math.Sin(polarCoords[1]);
			}

			public double EvaluateAt(XPoint point)
			{
				double[] polarCoords = commonData.CalcPolarCoordinates(point);
				return Math.Sqrt(polarCoords[0]) * Math.Sin(polarCoords[1] / 2.0) * Math.Sin(polarCoords[1]);
			}

			public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point) => 0;
		}

		public class Func3 : ICrackTipEnrichment
		{
			private readonly IsotropicBrittleTipEnrichments2DCaching commonData;

			public Func3(IsotropicBrittleTipEnrichments2DCaching commonData)
			{
				this.commonData = commonData;
			}

			public EvaluatedFunction EvaluateAllAt(XPoint point)
			{
				(double[] polarCoords, TipJacobiansExplicit jacobians) = commonData.CalcAll(point);
				double sqrtR = Math.Sqrt(polarCoords[0]);
				double cosTheta = Math.Cos(polarCoords[1]);
				double sinTheta = Math.Sin(polarCoords[1]);
				double cosThetaHalf = Math.Cos(polarCoords[1] / 2.0);
				double sinThetaHalf = Math.Sin(polarCoords[1] / 2.0);

				double value = sqrtR * cosThetaHalf * sinTheta;
				var gradientPolar = Vector.CreateFromArray(new double[]
				{
					0.5 / sqrtR * cosThetaHalf * sinTheta, sqrtR * (-0.5 * sinThetaHalf * sinTheta + cosThetaHalf * cosTheta)
				});

				Vector gradientGlobal = jacobians.TransformScalarFieldDerivativesLocalPolarToGlobalCartesian(gradientPolar);
				return new EvaluatedFunction(value, gradientGlobal.RawData);
			}

			public double EvaluateAt(XNode node)
			{
				double[] polarCoords = commonData.CalcPolarCoordinates(node);
				return Math.Sqrt(polarCoords[0]) * Math.Cos(polarCoords[1] / 2.0) * Math.Sin(polarCoords[1]);
			}

			public double EvaluateAt(XPoint point)
			{
				double[] polarCoords = commonData.CalcPolarCoordinates(point);
				return Math.Sqrt(polarCoords[0]) * Math.Cos(polarCoords[1] / 2.0) * Math.Sin(polarCoords[1]);
			}

			public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point) => 0;
		}

		private class PointPolarDataCache
		{
			public XPoint originalPoint;

			public double[] polarCoords;

			public TipJacobiansExplicit polarJacobians;

			public PointPolarDataCache(XPoint originalPoint, double[] polarCoords, TipJacobiansExplicit polarJacobians)
			{
				this.originalPoint = originalPoint;
				this.polarCoords = polarCoords;
				this.polarJacobians = polarJacobians;
			}
		}

		private class NodePolarDataCache
		{
			public XNode originalNode;

			public double[] polarCoords;

			public NodePolarDataCache(XNode originalNode, double[] polarCoords)
			{
				this.originalNode = originalNode;
				this.polarCoords = polarCoords;
			}
		}
	}
}
