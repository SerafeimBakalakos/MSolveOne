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
	public class NodesWithModifiedEnrichmentsObserver : IEnrichmentObserver
	{
		private readonly CrackStepNodesWithModifiedLevelSetObserver crackStepNodesWithModifiedLevelSetObserver;

		public NodesWithModifiedEnrichmentsObserver(
			CrackStepNodesWithModifiedLevelSetObserver crackStepNodesWithModifiedLevelSetObserver)
		{
			this.crackStepNodesWithModifiedLevelSetObserver = crackStepNodesWithModifiedLevelSetObserver;
		}

		public HashSet<XNode> NodesWithModifiedEnrichments { get; } = new HashSet<XNode>();

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies
			=> new IEnrichmentObserver[] { crackStepNodesWithModifiedLevelSetObserver };

		public void EndCurrentAnalysisIteration()
		{
			NodesWithModifiedEnrichments.UnionWith(crackStepNodesWithModifiedLevelSetObserver.StepNodesWithModifiedLevelSets);
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
	}
}
