using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	public class PreviousCrackTipNodesObserver : IEnrichmentObserver
	{
		public HashSet<XNode> PreviousCrackTipNodes { get; } = new HashSet<XNode>();

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies => Array.Empty<IEnrichmentObserver>();

		public void EndCurrentAnalysisIteration()
		{
			WriteToDebug();
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
			// Do not remove this node from the previous tip nodes. It is possible for a node to be enriched with tip function
			// for 2 consecutive iterations.
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			if (enrichment.EnrichmentFunctions[0] is ICrackTipEnrichment)
			{
				PreviousCrackTipNodes.Add(node);
			}
		}

		public void StartNewAnalysisIteration()
		{
			PreviousCrackTipNodes.Clear();
		}

		public void WriteToDebug()
		{
			var msg = new StringBuilder("Previous crack tip nodes:");
			foreach (XNode node in PreviousCrackTipNodes.OrderBy(n => n.ID))
			{
				msg.Append(" " + node.ID);
			}
			Debug.WriteLine(msg);
		}
	}
}
