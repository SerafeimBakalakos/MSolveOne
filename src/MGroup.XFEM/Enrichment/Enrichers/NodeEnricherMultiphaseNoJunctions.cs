using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Phases;

//TODO: Determine whether to use step or ridge enrichments based on if each interface is cohesive or coherent.
namespace MGroup.XFEM.Enrichment.Enrichers
{
	public class NodeEnricherMultiphaseNoJunctions : INodeEnricher
	{
		private readonly Func<IPhaseBoundary, IEnrichmentFunction> createEnrichment;
		private readonly PhaseGeometryModel geometricModel;
		private readonly ISingularityResolver singularityResolver;
		private readonly IDofType[] stdDofs;

		private Dictionary<EnrichmentItem, IPhaseBoundary> enrichments;

		public NodeEnricherMultiphaseNoJunctions(PhaseGeometryModel geometricModel, IDofType[] stdDofs, 
			Func<IPhaseBoundary, IEnrichmentFunction> createEnrichment, ISingularityResolver singularityResolver)
		{
			this.geometricModel = geometricModel;
			this.stdDofs = stdDofs;
			this.createEnrichment = createEnrichment;
			this.singularityResolver = singularityResolver;
		}

		public List<IEnrichmentObserver> Observers { get; } = new List<IEnrichmentObserver>();

		public static NodeEnricherMultiphaseNoJunctions CreateStructuralRidge(PhaseGeometryModel geometryModel, int dimension)
		{
			IDofType[] stdDofs;
			if (dimension == 1) stdDofs = new IDofType[] { StructuralDof.TranslationX };
			else if (dimension == 2) stdDofs = new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY };
			else if (dimension == 3)
			{
				stdDofs = new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ };
			}
			else throw new ArgumentException("Dimension must be 1, 2 or 3");

			Func<IPhaseBoundary, IEnrichmentFunction> createEnrichment = (boundary) => new RidgeEnrichment(boundary);
			var resolver = new NullSingularityResolver(); // ridge enrichment does not produce singularities in the stiffness
			return new NodeEnricherMultiphaseNoJunctions(geometryModel, stdDofs, createEnrichment, resolver);
		}

		public static NodeEnricherMultiphaseNoJunctions CreateStructuralStep(PhaseGeometryModel geometryModel, int dimension)
		{
			// The singularity resolver is usually not necessary, unlike cracks.
			return CreateStructuralStep(geometryModel, dimension, new NullSingularityResolver());
		}

		public static NodeEnricherMultiphaseNoJunctions CreateStructuralStep(PhaseGeometryModel geometryModel, int dimension,
			ISingularityResolver singularityResolver)
		{
			IDofType[] stdDofs;
			if (dimension == 1) stdDofs = new IDofType[] { StructuralDof.TranslationX };
			else if (dimension == 2) stdDofs = new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY };
			else if (dimension == 3)
			{
				stdDofs = new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ };
			}
			else throw new ArgumentException("Dimension must be 1, 2 or 3");

			Func<IPhaseBoundary, IEnrichmentFunction> createEnrichment = (boundary) => new PhaseStepEnrichment(boundary);
			return new NodeEnricherMultiphaseNoJunctions(geometryModel, stdDofs, createEnrichment, singularityResolver);
		}

		public static NodeEnricherMultiphaseNoJunctions CreateThermalRidge(PhaseGeometryModel geometryModel)
		{
			var stdDofs = new IDofType[] { ThermalDof.Temperature };
			Func<IPhaseBoundary, IEnrichmentFunction> createEnrichment = (boundary) => new RidgeEnrichment(boundary);
			var resolver = new NullSingularityResolver(); // ridge enrichment does not produce singularities in the stiffness
			return new NodeEnricherMultiphaseNoJunctions(geometryModel, stdDofs, createEnrichment, resolver);
		}

		public static NodeEnricherMultiphaseNoJunctions CreateThermalStep(PhaseGeometryModel geometryModel)
		{
			// The singularity resolver is usually not necessary, unlike cracks.
			return CreateThermalStep(geometryModel, new NullSingularityResolver());
		}

		public static NodeEnricherMultiphaseNoJunctions CreateThermalStep(PhaseGeometryModel geometryModel, 
			ISingularityResolver singularityResolver)
		{
			var stdDofs = new IDofType[] { ThermalDof.Temperature };
			Func<IPhaseBoundary, IEnrichmentFunction> createEnrichment = (boundary) => new PhaseStepEnrichment(boundary);
			return new NodeEnricherMultiphaseNoJunctions(geometryModel, stdDofs, createEnrichment, singularityResolver);
		}

		public void ApplyEnrichments()
		{
			foreach (IEnrichmentObserver observer in Observers)
			{
				observer.StartNewAnalysisIteration();
			}

			// Find nodes of elements interacting with each discontinuity. These nodes will potentially be enriched.
			var nodesPerEnrichment = new Dictionary<EnrichmentItem, HashSet<XNode>>();
			foreach (IPhase phase in geometricModel.Phases.Values)
			{
				if (phase is DefaultPhase) continue;
				foreach (IXMultiphaseElement element in phase.BoundaryElements)
				{
					foreach (IPhaseBoundary boundary in element.PhaseIntersections.Keys)
					{
						EnrichmentItem enrichment = boundary.StepEnrichment;
						bool exists = nodesPerEnrichment.TryGetValue(enrichment, out HashSet<XNode> nodesToEnrich);
						if (!exists)
						{
							nodesToEnrich = new HashSet<XNode>();
							nodesPerEnrichment[enrichment] = nodesToEnrich;
						}
						foreach (XNode node in element.Nodes) nodesToEnrich.Add(node);
					}
				}
			}

			// Enrich these nodes with the corresponding enrichment
			foreach (var enrichmentNodesPair in nodesPerEnrichment)
			{
				EnrichmentItem enrichment = enrichmentNodesPair.Key;
				HashSet<XNode> nodesToEnrich = enrichmentNodesPair.Value;
				IPhaseBoundary boundary = this.enrichments[enrichment];

				// Some of these nodes may need to not be enriched after all, to avoid singularities in the global stiffness matrix
				HashSet<XNode> rejectedNodes = 
					singularityResolver.FindStepEnrichedNodesToRemove(nodesToEnrich, boundary.Geometry);

				// Enrich the rest of them
				nodesToEnrich.ExceptWith(rejectedNodes);
				foreach (XNode node in nodesToEnrich)
				{
					EnrichNode(node, enrichment);
				}
			}

			foreach (IEnrichmentObserver observer in Observers)
			{
				observer.EndCurrentAnalysisIteration();
			}
		}

		public IEnumerable<EnrichmentItem> DefineEnrichments()
		{
			this.enrichments = new Dictionary<EnrichmentItem, IPhaseBoundary>();
			foreach (IPhase phase in geometricModel.Phases.Values)
			{
				if (phase is DefaultPhase) continue;
				if (phase.ExternalBoundaries.Count > 1)
				{
					throw new NotImplementedException("This node enricher assumes that each phase has a single boundary");
				}
				IPhaseBoundary boundary = phase.ExternalBoundaries[0];

				IEnrichmentFunction enrFunc = createEnrichment(boundary); // e.g. step, ramp or ridge enrichment
				var enrDofs = new IDofType[stdDofs.Length];
				for (int i = 0; i < stdDofs.Length; ++i) enrDofs[i] = new EnrichedDof(enrFunc, stdDofs[i]);
				var enrItem = new EnrichmentItem(this.enrichments.Count, new IEnrichmentFunction[] { enrFunc }, enrDofs);

				boundary.StepEnrichment = enrItem;
				this.enrichments[enrItem] = boundary;
			}

			return enrichments.Keys;
		}

		private void EnrichNode(XNode node, EnrichmentItem enrichment)
		{
			if (!node.Enrichments.Contains(enrichment))
			{
				node.Enrichments.Add(enrichment);
				foreach (IEnrichmentFunction enrichmentFunc in enrichment.EnrichmentFunctions)
				{
					double value = enrichmentFunc.EvaluateAt(node);
					node.EnrichmentFuncs[enrichmentFunc] = value;
				}
			}
		}
	}
}
