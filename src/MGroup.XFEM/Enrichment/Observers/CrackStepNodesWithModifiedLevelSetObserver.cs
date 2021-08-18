using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
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
		private readonly ExteriorLsmCrack[] allCracks;
		private readonly Dictionary<EnrichmentItem, ExteriorLsmCrack> enrichmentsToCracks 
			= new Dictionary<EnrichmentItem, ExteriorLsmCrack>();

		public CrackStepNodesWithModifiedLevelSetObserver(params ExteriorLsmCrack[] allCracks)
		{
			if (allCracks.Length == 0)
			{
				throw new ArgumentException("All cracks of the model must be passed in");
			}
			this.allCracks = allCracks;
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
			ExteriorLsmCrack crack = FindCrack(enrichment);
			if (crack == null)
			{
				return;
			}

			bool nodeExists = LevelSetsOfStepNodes[enrichment].TryGetValue(node, out double previousLevelSet);
			double newLevelSet = crack.LsmGeometry.LevelSets[node.ID];
			if (nodeExists && (newLevelSet != previousLevelSet))
			{
				StepNodesWithModifiedLevelSets.Add(node);
			}
			LevelSetsOfStepNodes[enrichment][node] = newLevelSet;
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			ExteriorLsmCrack crack = FindCrack(enrichment);
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

		private ExteriorLsmCrack FindCrack(EnrichmentItem enrichment)
		{
			if (enrichment.EnrichmentFunctions[0] is CrackStepEnrichment)
			{
				bool crackExists = enrichmentsToCracks.TryGetValue(enrichment, out ExteriorLsmCrack result);
				if (crackExists)
				{
					return result;
				}
				else
				{
					foreach (ExteriorLsmCrack crack in allCracks)
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
