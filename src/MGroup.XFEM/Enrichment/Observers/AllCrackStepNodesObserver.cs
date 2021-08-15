using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	public class AllCrackStepNodesObserver : IEnrichmentObserver_v2
	{
		public HashSet<XNode> AllCrackStepNodes { get; } = new HashSet<XNode>();

		public void EndCurrentAnalysisIteration()
		{
			WriteToDebug();
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

		public void WriteToDebug()
		{
			var msg = new StringBuilder("All crack step nodes:");
			foreach (XNode node in AllCrackStepNodes.OrderBy(n => n.ID))
			{
				msg.Append(" " + node.ID);
			}
			Debug.WriteLine(msg);
		}
	}
}
