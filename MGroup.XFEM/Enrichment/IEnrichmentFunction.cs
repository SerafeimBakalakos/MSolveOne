using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Enrichment
{
    public interface IEnrichmentFunction
    {
        double EvaluateAt(XNode node);

        double EvaluateAt(XPoint point);

        EvaluatedFunction EvaluateAllAt(XPoint point);

        /// <summary>
        /// Evaluates the jump of the enrichment function across a discontinuity. Every discontinuity has a positive and a 
        /// negative side. The jump is always considered as [[f(x)]] = f(x+) - f(x-).
        /// </summary>
        /// <param name="discontinuity"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point);
    }
}
