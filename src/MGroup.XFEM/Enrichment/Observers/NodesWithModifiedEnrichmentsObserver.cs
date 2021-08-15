using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	public class NodesWithModifiedEnrichmentsObserver : IEnrichmentObserver_v2
	{
		public HashSet<XNode> NodesWithModifiedEnrichments { get; } = new HashSet<XNode>();

		public void EndCurrentAnalysisIteration()
		{
			WriteToDebug();
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
			NodesWithModifiedEnrichments.Add(node);
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			NodesWithModifiedEnrichments.Add(node);
		}

		public void StartNewAnalysisIteration()
		{
			NodesWithModifiedEnrichments.Clear();
		}

		public void WriteToDebug()
		{
			var msg = new StringBuilder("Nodes with modified enrichments:");
			foreach (XNode node in NodesWithModifiedEnrichments.OrderBy(n => n.ID))
			{
				msg.Append(" " + node.ID);
			}
			Debug.WriteLine(msg);
		}
	}
}
