using System;
using System.Collections.Generic;
using System.Text;

//TODO: Move this, its implementations, singularity resolvers, etc to a sub-namespace.
namespace MGroup.XFEM.Enrichment.Enrichers
{
	public interface INodeEnricher
	{
		List<IEnrichmentObserver> Observers { get; } //TODO: perhaps this should be passed into ApplyEnrichments()

		void ApplyEnrichments();

		IEnumerable<EnrichmentItem> DefineEnrichments();
	}
}
