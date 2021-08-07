using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Output.EnrichmentObservers
{
    public class PreviousCrackTipNodesObserver : IEnrichmentObserver
    {
        private readonly ICrack crack;
        private readonly PreviousEnrichmentsObserver previousEnrichmentsObserver;

        public PreviousCrackTipNodesObserver(ICrack crack, PreviousEnrichmentsObserver previousEnrichmentsObserver)
        {
            this.crack = crack;
            this.previousEnrichmentsObserver = previousEnrichmentsObserver;
        }

        public HashSet<XNode> PreviousTipNodes { get; } = new HashSet<XNode>();

        public void Update(IEnumerable<EnrichmentItem> allEnrichments)
        {
            PreviousTipNodes.Clear();
            Dictionary<EnrichmentItem, XNode[]> previousEnrichments = previousEnrichmentsObserver.PreviousEnrichments;
            bool theyExist = previousEnrichments.TryGetValue(crack.CrackTipEnrichments, out XNode[] nodes);
            if (theyExist) PreviousTipNodes.UnionWith(nodes);
        }

        public IEnrichmentObserver[] RegisterAfterThese() => new IEnrichmentObserver[] { previousEnrichmentsObserver };
    }
}
