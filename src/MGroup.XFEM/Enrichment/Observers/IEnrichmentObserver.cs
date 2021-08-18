using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	public interface IEnrichmentObserver
	{
		IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies { get; }

		void EndCurrentAnalysisIteration();

		void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment);

		void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment);

		void StartNewAnalysisIteration();

	}
}
