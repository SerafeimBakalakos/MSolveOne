using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	public class NewCrackTipNodesObserver : IEnrichmentObserver
	{
		public HashSet<XNode> NewCrackTipNodes { get; } = new HashSet<XNode>();

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies => Array.Empty<IEnrichmentObserver>();

		public void EndCurrentAnalysisIteration()
		{
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
			if (enrichment.EnrichmentFunctions[0] is ICrackTipEnrichment)
			{
				NewCrackTipNodes.Add(node);
			}
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			if (enrichment.EnrichmentFunctions[0] is ICrackTipEnrichment)
			{
				NewCrackTipNodes.Remove(node);
			}
		}

		public void StartNewAnalysisIteration()
		{
			NewCrackTipNodes.Clear();
		}
	}
}
