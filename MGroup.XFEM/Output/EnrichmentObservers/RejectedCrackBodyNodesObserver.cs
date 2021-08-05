using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Output.EnrichmentObservers
{
    /// <summary>
    /// Tracks nodes that are not enriched with Heaviside functions, despite belonging to elements that intersect with the crack, 
    /// since that would cause singularities in the stiffness matrices.
    /// </summary>
    public class RejectedCrackBodyNodesObserver : IEnrichmentObserver
    {
        private readonly CrackBodyNodesObserver bodyNodesObserver;
        private readonly ICrack crack;
        private readonly NewCrackTipNodesObserver tipNodesObserver;

        public RejectedCrackBodyNodesObserver(ICrack crack, NewCrackTipNodesObserver tipNodesObserver, 
            CrackBodyNodesObserver bodyNodesObserver)
        {
            this.crack = crack;
            this.tipNodesObserver = tipNodesObserver;
            this.bodyNodesObserver = bodyNodesObserver;
        }

        public HashSet<XNode> RejectedHeavisideNodes = new HashSet<XNode>();

        public IEnrichmentObserver[] RegisterAfterThese() => new IEnrichmentObserver[] { tipNodesObserver, bodyNodesObserver };

        public void Update(IEnumerable<EnrichmentItem> allEnrichments)
        {
            RejectedHeavisideNodes.Clear();
            var bodyElements = new HashSet<IXCrackElement>(crack.IntersectedElements);
            bodyElements.UnionWith(crack.ConformingElements);
            foreach (IXCrackElement element in bodyElements)
            {
                foreach (XNode node in element.Nodes)
                {
                    if (!bodyNodesObserver.BodyNodes.Contains(node) && !tipNodesObserver.TipNodes.Contains(node))
                    {
                        RejectedHeavisideNodes.Add(node);
                    }
                }
            }
        }
    }
}
