using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	/// <summary>
	/// Tracks nodes with different enrichments or different values of enrichment functions between two consecutive
	/// crack propagation steps. These changes may result from application or removal of crack tip enrichments, application of 
	/// crack body enrichments or change in the values of crack body enrichments or level sets, due to the crack curving.
	/// </summary>
	public class NodesWithModifiedEnrichmentsObserver : IEnrichmentObserver_v2
	{
		public HashSet<XNode> NodesWithModifiedEnrichments { get; } = new HashSet<XNode>();

		public void EndCurrentAnalysisIteration()
		{
			throw new NotImplementedException("Also track if the level set has changed");
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
