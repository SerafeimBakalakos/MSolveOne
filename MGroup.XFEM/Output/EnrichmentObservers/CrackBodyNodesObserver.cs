using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Output.EnrichmentObservers
{
    public class CrackBodyNodesObserver : IEnrichmentObserver
    {
        private readonly ICrack crack;

        public CrackBodyNodesObserver(ICrack crack)
        {
            this.crack = crack;
        }

        public HashSet<XNode> BodyNodes { get; } = new HashSet<XNode>();

        public void Update(IEnumerable<EnrichmentItem> allEnrichments)
        {
            BodyNodes.Clear();
            BodyNodes.UnionWith(crack.CrackBodyEnrichment.EnrichedNodes);
        }
        
        public IEnrichmentObserver[] RegisterAfterThese() => new IEnrichmentObserver[0];
    }
}
