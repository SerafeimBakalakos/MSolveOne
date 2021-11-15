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

		//TODO: perhaps these 3 are redundant, since enrichment addition and removal is being tracked.
		private readonly NewCrackStepNodesObserver newCrackStepNodesObserver;
		private readonly NewCrackTipNodesObserver newCrackTipNodesObserver;
		private readonly PreviousCrackTipNodesObserver previousCrackTipNodesObserver;

		public NodesWithModifiedEnrichmentsObserver(NewCrackStepNodesObserver newCrackStepNodesObserver,
			CrackStepNodesWithModifiedLevelSetObserver crackStepNodesWithModifiedLevelSetObserver, 
			NewCrackTipNodesObserver newCrackTipNodesObserver, PreviousCrackTipNodesObserver previousCrackTipNodesObserver)
		{
			this.newCrackStepNodesObserver = newCrackStepNodesObserver;
			this.crackStepNodesWithModifiedLevelSetObserver = crackStepNodesWithModifiedLevelSetObserver;
			this.newCrackTipNodesObserver = newCrackTipNodesObserver;
			this.previousCrackTipNodesObserver = previousCrackTipNodesObserver;
		}

		public HashSet<XNode> NodesWithModifiedEnrichments { get; } = new HashSet<XNode>();

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies
			=> new IEnrichmentObserver[] 
			{ 
				newCrackStepNodesObserver, crackStepNodesWithModifiedLevelSetObserver, 
				newCrackTipNodesObserver, previousCrackTipNodesObserver
			};

		public void EndCurrentAnalysisIteration()
		{
			NodesWithModifiedEnrichments.UnionWith(newCrackStepNodesObserver.NewCrackStepNodes);
			NodesWithModifiedEnrichments.UnionWith(crackStepNodesWithModifiedLevelSetObserver.StepNodesWithModifiedLevelSets);
			NodesWithModifiedEnrichments.UnionWith(newCrackTipNodesObserver.NewCrackTipNodes);
			NodesWithModifiedEnrichments.UnionWith(previousCrackTipNodesObserver.PreviousCrackTipNodes);
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
