using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	/// <summary>
	/// Tracks elements with nodes with different enrichments or different values of enrichment functions between two consecutive
	/// crack propagation steps.    
	/// </summary>
	public class ElementsWithModifiedNodesObserver : IEnrichmentObserver
	{
		private readonly NodesWithModifiedEnrichmentsObserver nodesWithModifiedEnrichmentsObserver;

		public ElementsWithModifiedNodesObserver(NodesWithModifiedEnrichmentsObserver nodesWithModifiedEnrichmentsObserver)
		{
			this.nodesWithModifiedEnrichmentsObserver = nodesWithModifiedEnrichmentsObserver;
		}

		public HashSet<IXFiniteElement> ModifiedElements { get; } = new HashSet<IXFiniteElement>();

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies 
			=> new IEnrichmentObserver[] { nodesWithModifiedEnrichmentsObserver };

		public void EndCurrentAnalysisIteration()
		{
			foreach (XNode node in nodesWithModifiedEnrichmentsObserver.NodesWithModifiedEnrichments)
			{
				ModifiedElements.UnionWith(node.ElementsDictionary.Values);
			}
			WriteToDebug();
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
		}

		public void StartNewAnalysisIteration()
		{
			ModifiedElements.Clear();
		}

		public void WriteToDebug()
		{
			var msg = new StringBuilder("Elements with modified nodes:");
			foreach (IXFiniteElement element in ModifiedElements.OrderBy(n => n.ID))
			{
				msg.Append(" " + element.ID);
			}
			Debug.WriteLine(msg);
		}
	}
}
