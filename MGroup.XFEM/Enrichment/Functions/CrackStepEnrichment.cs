using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Enrichment.Functions
{
    public class CrackStepEnrichment : IEnrichmentFunction
    {
        private readonly ICrack crack;
        private readonly IXGeometryDescription crackGeometry;

        public CrackStepEnrichment(ICrack crack)
        {
            this.crack = crack;
            this.crackGeometry = crack.CrackGeometry;
        }

        public EvaluatedFunction EvaluateAllAt(XPoint point)
        {
            double distance = crackGeometry.SignedDistanceOf(point);
            if (distance >= 0) return new EvaluatedFunction(+1, new double[2]);
            else return new EvaluatedFunction(-1, new double[2]);
        }

        public double EvaluateAt(XNode node)
        {
            double distance = crackGeometry.SignedDistanceOf(node);
            if (distance >= 0) return +1;
            else return -1;
        }

        public double EvaluateAt(XPoint point)
        {
            double distance = crackGeometry.SignedDistanceOf(point);
            if (distance >= 0) return +1;
            else return -1;
        }

        public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point)
        {
            if (discontinuity == crack) return 2.0; // +1 - (-1)
            else return 0.0;
        }
    }
}
