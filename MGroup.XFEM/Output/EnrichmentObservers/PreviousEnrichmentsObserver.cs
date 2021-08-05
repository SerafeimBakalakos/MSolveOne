using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Output.EnrichmentObservers
{
    /// <summary>
    /// This may substantially increase the memory requirements, since it stores two copies of the enrichments-nodes 
    /// relationships.
    /// </summary>
    public class PreviousEnrichmentsObserver : IEnrichmentObserver
    {
        public Dictionary<EnrichmentItem, XNode[]> PreviousEnrichments { get; set; } = new Dictionary<EnrichmentItem, XNode[]>();
        private Dictionary<EnrichmentItem, XNode[]> CurrentEnrichments { get; set; } = new Dictionary<EnrichmentItem, XNode[]>();

        public void Update(IEnumerable<EnrichmentItem> allEnrichments)
        {
            PreviousEnrichments = CurrentEnrichments;

            // Store current enrichments
            CurrentEnrichments = new Dictionary<EnrichmentItem, XNode[]>();
            foreach (EnrichmentItem enrichment in allEnrichments)
            {
                CurrentEnrichments[enrichment] = enrichment.EnrichedNodes.ToArray();
            }
        }

        public IEnrichmentObserver[] RegisterAfterThese() => new IEnrichmentObserver[0];
    }
}
