using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	/// <summary>
	/// Tracks nodes that are not enriched with Heaviside functions, despite belonging to elements that intersect with the crack, 
	/// since that would cause singularities in the stiffness matrices.
	/// </summary>
	public class RejectedCrackStepNodesObserver : IEnrichmentObserver
	{
		private readonly ICrack crack;
		private readonly NewCrackTipNodesObserver tipNodesObserver;
		private readonly AllCrackStepNodesObserver stepNodesObserver;

		public RejectedCrackStepNodesObserver(ICrack crack, NewCrackTipNodesObserver tipNodesObserver,
			AllCrackStepNodesObserver stepNodesObserver)
		{
			this.crack = crack;
			this.tipNodesObserver = tipNodesObserver;
			this.stepNodesObserver = stepNodesObserver;
		}

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies
			=> new IEnrichmentObserver[] { tipNodesObserver, stepNodesObserver };

		public HashSet<XNode> RejectedHeavisideNodes { get; } = new HashSet<XNode>();

		public void EndCurrentAnalysisIteration()
		{
			var bodyElements = new HashSet<IXCrackElement>(crack.IntersectedElements);
			bodyElements.UnionWith(crack.ConformingElements);
			foreach (IXCrackElement element in bodyElements)
			{
				foreach (XNode node in element.Nodes)
				{
					if (!stepNodesObserver.AllCrackStepNodes.Contains(node) && !tipNodesObserver.NewCrackTipNodes.Contains(node))
					{
						RejectedHeavisideNodes.Add(node);
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
			RejectedHeavisideNodes.Clear();
		}
	}
}
