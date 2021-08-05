using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Output.EnrichmentObservers
{
    public class NewCrackBodyNodesObserver : IEnrichmentObserver
    {
        private readonly ICrack crack;
        private readonly PreviousEnrichmentsObserver previousEnrichmentsObserver;
        private readonly CrackBodyNodesObserver bodyNodesObserver;

        public NewCrackBodyNodesObserver(ICrack crack, PreviousEnrichmentsObserver previousEnrichmentsObserver, 
            CrackBodyNodesObserver bodyNodesObserver)
        {
            this.crack = crack;
            this.previousEnrichmentsObserver = previousEnrichmentsObserver;
            this.bodyNodesObserver = bodyNodesObserver;
        }

        public HashSet<XNode> NewBodyNodes { get; } = new HashSet<XNode>();

        public void Update(IEnumerable<EnrichmentItem> allEnrichments)
        {
            NewBodyNodes.Clear();

            var previousBodyNodes = new HashSet<XNode>();
            bool theyExist = previousEnrichmentsObserver.PreviousEnrichments.TryGetValue(
                crack.CrackBodyEnrichment, out XNode[] nodes);
            if (theyExist) previousBodyNodes.UnionWith(nodes);

            foreach (XNode node in bodyNodesObserver.BodyNodes)
            {
                if (!previousBodyNodes.Contains(node)) NewBodyNodes.Add(node);
            }
        }

        public IEnrichmentObserver[] RegisterAfterThese()
        {
            return new IEnrichmentObserver[] { previousEnrichmentsObserver, bodyNodesObserver };
        }
    }
}
