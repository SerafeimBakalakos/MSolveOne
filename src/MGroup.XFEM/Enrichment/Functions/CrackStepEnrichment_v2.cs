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
	public class CrackStepEnrichment_v2 : IEnrichmentFunction
	{
		private readonly IImplicitCrackGeometry crack;

		public CrackStepEnrichment_v2(IImplicitCrackGeometry crack)
		{
			this.crack = crack;
		}

		public EvaluatedFunction EvaluateAllAt(XPoint point)
		{
			//TODO: cache this inside XPoint
			double[] levelSets = crack.InterpolateLevelSets(point);

			double phi = levelSets[0];
			if (phi >= 0)
			{
				return new EvaluatedFunction(+1, new double[point.Dimension]);
			}
			else
			{
				return new EvaluatedFunction(-1, new double[point.Dimension]);
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
				return 2.0; // +1 - (-1)
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
				return +1;
			}
			else
			{
				return -1;
			}
		}

	}
}
