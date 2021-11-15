using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	/// <summary>
	/// Tracks nodes that are enriched with crack body enrichments in this and the previous step, but their corresponding crack 
	/// body level sets were modified with respect to their values during the previous propagation step.
	/// </summary>
	public class CrackStepNodesWithModifiedLevelSetObserver : IEnrichmentObserver
	{
		private readonly ICrack[] allCracks;
		private readonly Dictionary<EnrichmentItem, ICrack> enrichmentsToCracks = new Dictionary<EnrichmentItem, ICrack>();

		public CrackStepNodesWithModifiedLevelSetObserver(XModel<IXCrackElement> model)
		{
			var allCracks = new List<ICrack>();
			foreach (IXDiscontinuity discontinuity in model.GeometryModel.EnumerateDiscontinuities())
			{
				if (discontinuity is ICrack crack)
				{
					allCracks.Add(crack);
				}
			}
			this.allCracks = allCracks.ToArray();
		}

		public Dictionary<EnrichmentItem, Dictionary<XNode, double>> LevelSetsOfStepNodes { get; }
			= new Dictionary<EnrichmentItem, Dictionary<XNode, double>>();

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies => Array.Empty<IEnrichmentObserver>();

		/// <summary>
		/// These may be different from the crack body nodes provided by the cracks themselves, since they may correspond to the
		/// previous propagation step, depending on when they are accessed.
		/// </summary>
		public HashSet<XNode> StepNodesWithModifiedLevelSets { get; } = new HashSet<XNode>();

		public void EndCurrentAnalysisIteration()
		{
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
			ICrack crack = FindCrack(enrichment);
			if (crack == null)
			{
				return;
			}

			bool nodeExists = LevelSetsOfStepNodes[enrichment].TryGetValue(node, out double previousLevelSet);
			double newLevelSet = crack.CrackGeometry.SignedDistanceOf(node);
			if (nodeExists && (newLevelSet != previousLevelSet))
			{
				StepNodesWithModifiedLevelSets.Add(node);
			}
			LevelSetsOfStepNodes[enrichment][node] = newLevelSet;
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			ICrack crack = FindCrack(enrichment);
			if (crack == null)
			{
				return;
			}

			LevelSetsOfStepNodes[enrichment].Remove(node);
		}

		public void StartNewAnalysisIteration()
		{
			StepNodesWithModifiedLevelSets.Clear();
		}

		private ICrack FindCrack(EnrichmentItem enrichment)
		{
			if (enrichment.EnrichmentFunctions[0] is IStepEnrichment)
			{
				bool crackExists = enrichmentsToCracks.TryGetValue(enrichment, out ICrack result);
				if (crackExists)
				{
					return result;
				}
				else
				{
					foreach (ICrack crack in allCracks)
					{
						if (crack.CrackBodyEnrichment == enrichment)
						{
							enrichmentsToCracks[enrichment] = crack;
							LevelSetsOfStepNodes[enrichment] = new Dictionary<XNode, double>();
							return crack;
						}
					}
					throw new KeyNotFoundException(
						"Found crack step enrichment that does not correspond to any of the known cracks.");
				}
			}
			else
			{
				return null;
			}
		}
	}
}
