using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	public class NodesWithModifiedEnrichmentsObserver : IEnrichmentObserver_v2
	{
		private readonly HashSet<int> nodes = new HashSet<int>();

		public void IncrementAnalysisIteration()
		{
			WriteToDebug();
			nodes.Clear();
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
			nodes.Add(node.ID);
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			nodes.Add(node.ID);
		}

		public void WriteToDebug()
		{
			var msg = new StringBuilder("Nodes with modified enrichments:");
			foreach (int node in nodes.OrderBy(n => n))
			{
				msg.Append(" " + node);
			}
			Debug.WriteLine(msg);
		}
	}
}