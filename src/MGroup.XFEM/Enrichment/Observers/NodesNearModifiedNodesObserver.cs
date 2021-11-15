using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	/// <summary>
	/// Nodes that have unmodified enrichments, but belong to the same elements as nodes that have modified enrichments.
	/// </summary>
	public class NodesNearModifiedNodesObserver : IEnrichmentObserver
	{
		private readonly NodesWithModifiedEnrichmentsObserver nodesWithModifiedEnrichmentsObserver;
		private readonly ElementsWithModifiedNodesObserver elementsWithModifiedNodesObserver;

		public NodesNearModifiedNodesObserver(NodesWithModifiedEnrichmentsObserver nodesWithModifiedEnrichmentsObserver,
			ElementsWithModifiedNodesObserver elementsWithModifiedNodesObserver)
		{
			this.nodesWithModifiedEnrichmentsObserver = nodesWithModifiedEnrichmentsObserver;
			this.elementsWithModifiedNodesObserver = elementsWithModifiedNodesObserver;
		}

		public HashSet<XNode> NearModifiedNodes { get; } = new HashSet<XNode>();

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies
			=> new IEnrichmentObserver[] { nodesWithModifiedEnrichmentsObserver, elementsWithModifiedNodesObserver };

		public void EndCurrentAnalysisIteration()
		{
			HashSet<XNode> modifiedNodes = nodesWithModifiedEnrichmentsObserver.NodesWithModifiedEnrichments;
			foreach (IXFiniteElement element in elementsWithModifiedNodesObserver.ModifiedElements)
			{
				foreach (XNode node in element.Nodes)
				{
					if ((!modifiedNodes.Contains(node)) && (node.EnrichmentFuncs.Count > 0))
					{
						NearModifiedNodes.Add(node);
					}
				}
			}
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
		}

		public void StartNewAnalysisIteration()
		{
			NearModifiedNodes.Clear();
		}
	}
}
