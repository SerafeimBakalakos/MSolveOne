using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Enrichment.Functions
{
    public class PhaseStepEnrichment : IEnrichmentFunction
    {
        private readonly IPhaseBoundary boundary;

        public PhaseStepEnrichment(IPhaseBoundary boundary)
        {
            this.boundary = boundary;
        }

        public EvaluatedFunction EvaluateAllAt(XPoint point)
        {
            var gradient = new double[point.Dimension];
            double signedDistance = boundary.Geometry.SignedDistanceOf(point);
            if (signedDistance >= 0) return new EvaluatedFunction(+1.0, gradient);
            else return new EvaluatedFunction(-1.0, gradient);
        }

        public double EvaluateAt(XNode node)
        {
            double signedDistance = boundary.Geometry.SignedDistanceOf(node);
            if (signedDistance >= 0) return +1.0;
            else return -1.0;
        }

        public double EvaluateAt(XPoint point)
        {
            double signedDistance = boundary.Geometry.SignedDistanceOf(point);
            if (signedDistance >= 0) return +1.0;
            else return -1.0;
        }

        public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point)
        {
            if (discontinuity == this.boundary) return 2.0; // +1 - (-1)
            else return 0.0;
        }
    }
}
