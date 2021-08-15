using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment
{
	public interface IEnrichmentObserver_v2
	{
		void IncrementAnalysisIteration();

		void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment);

		void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment);
	}
}
