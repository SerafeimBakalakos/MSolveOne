using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	public class NewCrackTipNodesObserver : IEnrichmentObserver_v2
	{
		public HashSet<XNode> NewCrackTipNodes { get; } = new HashSet<XNode>();

		public void EndCurrentAnalysisIteration()
		{
			WriteToDebug();
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

		public void WriteToDebug()
		{
			var msg = new StringBuilder("New crack tip nodes:");
			foreach (XNode node in NewCrackTipNodes.OrderBy(n => n.ID))
			{
				msg.Append(" " + node.ID);
			}
			Debug.WriteLine(msg);
		}
	}
}
