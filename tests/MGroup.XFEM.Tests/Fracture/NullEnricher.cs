using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.Observers;

namespace MGroup.XFEM.Tests.Fracture
{
	public class NullEnricher : INodeEnricher
	{
		public List<IEnrichmentObserver> Observers { get; } = new List<IEnrichmentObserver>();

		public void ApplyEnrichments()
		{
		}

		public IEnumerable<EnrichmentItem> DefineEnrichments()
		{
			return new EnrichmentItem[0];
		}
	}
}
