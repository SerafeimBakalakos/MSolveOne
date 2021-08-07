using System;
using System.Collections.Generic;
using System.Text;

//TODO: Move this, its implementations, singularity resolvers, etc to a sub-namespace.
namespace MGroup.XFEM.Enrichment.Enrichers
{
    public interface INodeEnricher
    {
        void ApplyEnrichments();

        IEnumerable<EnrichmentItem> DefineEnrichments();
    }
}
