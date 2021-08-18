using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	public class NewCrackStepNodesObserver : IEnrichmentObserver
	{
		public HashSet<XNode> NewCrackStepNodes { get; } = new HashSet<XNode>();

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies => Array.Empty<IEnrichmentObserver>();

		public void EndCurrentAnalysisIteration()
		{
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
			if (enrichment.EnrichmentFunctions[0] is CrackStepEnrichment)
			{
				NewCrackStepNodes.Add(node);
			}
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			if (enrichment.EnrichmentFunctions[0] is CrackStepEnrichment)
			{
				NewCrackStepNodes.Remove(node);
			}
		}

		public void StartNewAnalysisIteration()
		{
			NewCrackStepNodes.Clear();
		}
	}
}
