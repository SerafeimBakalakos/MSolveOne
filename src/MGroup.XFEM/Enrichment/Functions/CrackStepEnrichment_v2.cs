using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Enrichment.Functions
{
	public class CrackStepEnrichment_v2 : IStepEnrichment
	{
		private readonly IImplicitCrackGeometry crack;
		private readonly double valuePos, valueNeg;

		public CrackStepEnrichment_v2(IImplicitCrackGeometry crack, bool oppositeSign = false)
		{
			this.crack = crack;
			if (oppositeSign)
			{
				valuePos = -1.0;
				valueNeg = +1.0;
			}
			else
			{
				valuePos = +1.0;
				valueNeg = -1.0;
			}
		}

		public EvaluatedFunction EvaluateAllAt(XPoint point)
		{
			//TODO: cache this inside XPoint
			double[] levelSets = crack.InterpolateLevelSets(point);

			double phi = levelSets[0];
			if (phi >= 0)
			{
				return new EvaluatedFunction(valuePos, new double[point.Dimension]);
			}
			else
			{
				return new EvaluatedFunction(valueNeg, new double[point.Dimension]);
			}
		}

		public double EvaluateAt(XNode node)
		{
			double[] levelSets = crack.GetNodalLevelSets(node);
			return EvaluateAt(levelSets);
		}

		public double EvaluateAt(XPoint point)
		{
			//TODO: cache this inside XPoint
			double[] levelSets = crack.InterpolateLevelSets(point);
			return EvaluateAt(levelSets);
		}


		public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point)
		{
			if (discontinuity == crack)
			{
				return valuePos - valueNeg;
			}
			else
			{
				return 0.0;
			}
		}

		private double EvaluateAt(double[] levelSets)
		{
			double phi = levelSets[0];
			if (phi >= 0)
			{
				return valuePos;
			}
			else
			{
				return valueNeg;
			}
		}

	}
}
