using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Enrichment.Functions
{
	public class MockStepEnrichment : IEnrichmentFunction
	{
		public EvaluatedFunction EvaluateAllAt(XPoint point) => new EvaluatedFunction(0, new double[point.Dimension]);

		public double EvaluateAt(XNode node) => 0;

		public double EvaluateAt(XPoint point) => 0;

		public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point)
		{
			throw new NotImplementedException();
		}
	}
}
