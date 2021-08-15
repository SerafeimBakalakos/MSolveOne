using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Enrichment.Enrichers
{
	public class NodeEnricherIndependentCracks : INodeEnricher
	{
		private readonly double fixedTipEnrichmentRegionRadius;
		private readonly CrackGeometryModel geometryModel;
		private readonly ISingularityResolver singularityResolver;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="fixedTipEnrichmentRegionRadius">
		/// If a fixed enrichment area is applied, all nodes inside a circle around the tip are enriched with tip 
		/// functions. They can still be enriched with Heaviside functions, if they do not belong to the tip 
		/// element(s).
		/// </param>
		public NodeEnricherIndependentCracks(CrackGeometryModel geometryModel,
			ISingularityResolver singularityResolver, double fixedTipEnrichmentRegionRadius = 0.0)
		{
			this.geometryModel = geometryModel;
			this.singularityResolver = singularityResolver;
			this.fixedTipEnrichmentRegionRadius = fixedTipEnrichmentRegionRadius;
		}

		public List<IEnrichmentObserver_v2> Observers { get; } = new List<IEnrichmentObserver_v2>();

		/// <summary>
		/// 
		/// </summary>
		/// <param name="fixedTipEnrichmentRegionRadius">
		/// If a fixed enrichment area is applied, all nodes inside a circle around the tip are enriched with tip 
		/// functions. They can still be enriched with Heaviside functions, if they do not belong to the tip 
		/// element(s).
		/// </param>
		public NodeEnricherIndependentCracks(CrackGeometryModel geometryModel, double fixedTipEnrichmentRegionRadius = 0.0) :
			this(geometryModel, new RelativeAreaSingularityResolver(1E-4), fixedTipEnrichmentRegionRadius)
		{
		}

		public void ApplyEnrichments()
		{
			foreach (IEnrichmentObserver_v2 observer in Observers)
			{
				observer.IncrementAnalysisIteration();
			}

			foreach (ICrack crack in geometryModel.Cracks.Values)
			{
				// Remove crack tip enrichments of previous configuration
				ClearNodalEnrichments(crack.CrackTipEnrichments);

				// Enrich nodes of the new crack tip elements
				var tipElementNodes = new HashSet<XNode>();
				foreach (IXCrackElement element in crack.TipElements)
				{
					tipElementNodes.UnionWith(element.Nodes);
				}
				EnrichNodesWith(tipElementNodes, crack.CrackTipEnrichments);

				// Extra tip nodes due to "fixed tip enrichment area"
				if (fixedTipEnrichmentRegionRadius > 0.0) 
				{
					var circle = new Circle2D(crack.TipCoordinates, fixedTipEnrichmentRegionRadius); //TODO: This needs adapting for 3D
					HashSet<XNode> extraTipNodes = MeshUtilities.FindNodesInsideCircle(circle, crack.TipElements.First());
					extraTipNodes.ExceptWith(tipElementNodes);
					EnrichNodesWith(extraTipNodes, crack.CrackTipEnrichments);
				}

				// Heaviside nodes
				var heavisideNodes = new HashSet<XNode>();
				foreach (IXCrackElement element in crack.IntersectedElements)
				{
					heavisideNodes.UnionWith(element.Nodes);
				}
				foreach (IXCrackElement element in crack.ConformingElements)
				{
					foreach (XNode node in element.Nodes)
					{
						double distance = crack.CrackGeometry.SignedDistanceOf(node);
						if (distance == 0.0) //TODO: what if the geometry class uses some tolerance for this? Is this reflected in .SignedDistanceOf(node)?
						{
							heavisideNodes.Add(node);
						}
					}
				}

				// Do not enrich the nodes of the crack tip(s)
				heavisideNodes.ExceptWith(tipElementNodes);

				// Also do not enrich nodes that may cause singularities
				HashSet<XNode> rejectedNodes = 
					singularityResolver.FindStepEnrichedNodesToRemove(heavisideNodes, crack.CrackGeometry);
				heavisideNodes.ExceptWith(rejectedNodes);

				// Only enrich the new Heaviside nodes, namely the ones not previously enriched. This will cause problems if the 
				// crack tip turns sharply towards the crack body, which shouldn't happen normally.
				//TODO: This optimization is not necessary, but observers may depend on it. 
				var newHeavisideNodes = new HashSet<XNode>();
				foreach (XNode node in heavisideNodes)
				{
					if (!node.Enrichments.Contains(crack.CrackBodyEnrichment))
					{
						newHeavisideNodes.Add(node);
					}
				}
				EnrichNodesWith(newHeavisideNodes, crack.CrackBodyEnrichment);
			}
		}

		public IEnumerable<EnrichmentItem> DefineEnrichments()
		{
			var enrichmentItems = new List<EnrichmentItem>();
			foreach (ICrack crack in geometryModel.Cracks.Values)
			{
				IList<EnrichmentItem> crackEnrichments = crack.DefineEnrichments(enrichmentItems.Count);
				enrichmentItems.AddRange(crackEnrichments);
			}
			return enrichmentItems;
		}

		private void ClearNodalEnrichments(EnrichmentItem enrichment)
		{
			foreach (XNode node in enrichment.EnrichedNodes)
			{
				node.Enrichments.Remove(enrichment);
				foreach (IEnrichmentFunction enrichmentFunc in enrichment.EnrichmentFunctions)
				{
					node.EnrichmentFuncs.Remove(enrichmentFunc);
				}

				foreach (IEnrichmentObserver_v2 observer in Observers)
				{
					observer.LogEnrichmentRemoval(node, enrichment);
				}
			}
			enrichment.EnrichedNodes.Clear();
		}

		private void EnrichNodesWith(IEnumerable<XNode> nodes, EnrichmentItem enrichment)
		{
			foreach (XNode node in nodes)
			{
				node.Enrichments.Add(enrichment);
				foreach (IEnrichmentFunction enrichmentFunc in enrichment.EnrichmentFunctions)
				{
					node.EnrichmentFuncs[enrichmentFunc] = enrichmentFunc.EvaluateAt(node);
				}
				enrichment.EnrichedNodes.Add(node);

				foreach (IEnrichmentObserver_v2 observer in Observers)
				{
					observer.LogEnrichmentAddition(node, enrichment);
				}
			}
		}
	}
}
