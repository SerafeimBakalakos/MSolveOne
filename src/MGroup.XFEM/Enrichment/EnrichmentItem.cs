using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.XFEM.Entities;

//TODO: This can also be used for common operations between crack tip functions. It will need abstracting though and may not be the best idea.
//TODO: Delete IEnrichmentFunction.ID and in implementations
//TODO: Rename to Enrichment and probably move to entities
namespace MGroup.XFEM.Enrichment
{
    public class EnrichmentItem
    {
        public EnrichmentItem(int id, IEnrichmentFunction[] enrichmentFunctions, IDofType[] enrichedDofs)
        {
            this.ID = id;
            this.EnrichmentFunctions = enrichmentFunctions;
            this.EnrichedDofs = enrichedDofs;
        }

        public int ID { get; }

        public IDofType[] EnrichedDofs { get; }

        public IEnrichmentFunction[] EnrichmentFunctions { get; }

        public HashSet<XNode> EnrichedNodes { get; } = new HashSet<XNode>();

        public override int GetHashCode() => ID.GetHashCode();
    }
}
