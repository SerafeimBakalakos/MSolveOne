using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	public class AllCrackStepNodesObserver : IEnrichmentObserver
	{
		public HashSet<XNode> AllCrackStepNodes { get; } = new HashSet<XNode>();

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies => Array.Empty<IEnrichmentObserver>();

		public void EndCurrentAnalysisIteration()
		{
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
			if (enrichment.EnrichmentFunctions[0] is CrackStepEnrichment)
			{
				AllCrackStepNodes.Add(node);
			}
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			if (enrichment.EnrichmentFunctions[0] is CrackStepEnrichment)
			{
				AllCrackStepNodes.Remove(node);
			}
		}

		public void StartNewAnalysisIteration()
		{
		}
	}
}
