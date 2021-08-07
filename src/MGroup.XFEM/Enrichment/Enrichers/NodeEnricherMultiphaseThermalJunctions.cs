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

//TODO: Remove casts
//TODO: Determine whether to use step or ridge enrichments based on if each interface is cohesive or coherent.
//TODO: Resolving singularities is only needed in step enrichment (cohesive interfaces).
namespace MGroup.XFEM.Enrichment.Enrichers
{
    public class NodeEnricherMultiphaseThermalJunctions : INodeEnricher
    {
        private readonly PhaseGeometryModel geometricModel;
        private readonly ISingularityResolver singularityResolver;

        public NodeEnricherMultiphaseThermalJunctions(PhaseGeometryModel geometricModel)
            : this(geometricModel, new NullSingularityResolver())
        {
        }

        public NodeEnricherMultiphaseThermalJunctions(PhaseGeometryModel geometricModel, 
            ISingularityResolver singularityResolver)
        {
            this.geometricModel = geometricModel;
            this.singularityResolver = singularityResolver;
            this.JunctionElements = new Dictionary<IXFiniteElement, HashSet<JunctionEnrichment>>();
        }

        public Dictionary<IXFiniteElement, HashSet<JunctionEnrichment>> JunctionElements { get; }

        public void ApplyEnrichments()
        {
            // Junction enrichments
            foreach (var elementJunctionPair in JunctionElements)
            {
                IXFiniteElement element = elementJunctionPair.Key;
                foreach (JunctionEnrichment junctionEnrichment in elementJunctionPair.Value)
                {
                    foreach (XNode node in element.Nodes) EnrichNode(node, junctionEnrichment);
                }
            }

            // Find nodes to potentially be enriched by step enrichments
            var nodesPerStepEnrichment = new Dictionary<EnrichmentItem, HashSet<XNode>>();
            foreach (IPhase phase in geometricModel.Phases.Values)
            {
                if (phase is DefaultPhase) continue;
                foreach (IXMultiphaseElement element in phase.BoundaryElements)
                {
                    foreach (IPhaseBoundary boundary in element.PhaseIntersections.Keys)
                    {
                        // Find the nodes to potentially be enriched by this step enrichment 
                        EnrichmentItem stepEnrichment = boundary.StepEnrichment;
                        var stepEnrichmentFunc = (PhaseStepEnrichmentOLD)stepEnrichment.EnrichmentFunctions[0]; //TODO: remove the cast
                        bool exists = nodesPerStepEnrichment.TryGetValue(stepEnrichment, out HashSet<XNode> nodesToEnrich);
                        if (!exists)
                        {
                            nodesToEnrich = new HashSet<XNode>();
                            nodesPerStepEnrichment[stepEnrichment] = nodesToEnrich;
                        }

                        // Only enrich a node if it does not have a corresponding junction enrichment
                        foreach (XNode node in element.Nodes)
                        {
                            if (!HasCorrespondingJunction(node, stepEnrichmentFunc)) nodesToEnrich.Add(node);
                        }
                    }
                }
            }

            // Enrich these nodes with the corresponding step enrichment
            foreach (var enrichmentNodesPair in nodesPerStepEnrichment)
            {
                EnrichmentItem stepEnrichment = enrichmentNodesPair.Key;
                HashSet<XNode> nodesToEnrich = enrichmentNodesPair.Value;

                // Some of these nodes may need to not be enriched after all, to avoid singularities in the global stiffness matrix
                //HashSet<XNode> rejectedNodes = singularityResolver.FindStepEnrichedNodesToRemove(nodesToEnrich, stepEnrichment);

                // Enrich the rest of them
                //nodesToEnrich.ExceptWith(rejectedNodes);
                foreach (XNode node in nodesToEnrich)
                {
                    EnrichNode(node, stepEnrichment);
                }
            }
        }

        public IEnumerable<EnrichmentItem> DefineEnrichments()
        {
            var enrichmentItems = new List<EnrichmentItem>();
            DefineStepEnrichments(enrichmentItems);
            DefineJunctionEnrichments(enrichmentItems);
            return enrichmentItems;
        }

        private static (IPhase minPhase, IPhase maxPhase) FindMinMaxPhases(IPhase phase1, IPhase phase2)
        {
            IPhase minPhase, maxPhase;
            if (phase1.ID < phase2.ID)
            {
                minPhase = phase1;
                maxPhase = phase2;
            }
            else
            {
                minPhase = phase2;
                maxPhase = phase1;
            }
            return (minPhase, maxPhase);
        }

        private void DefineJunctionEnrichments(List<EnrichmentItem> enrichmentItems)
        { //WARNING: Assumes 0 or 1 junction per element

            // Keep track of the junctions to avoid duplicate ones.
            //TODO: What happens if the same boundary is used for more than one junctions? E.g. the boundary is an almost closed 
            //      curve, that has 2 ends, both of which need junctions.
            var junctionEnrichments = new Dictionary<ClosedPhaseBoundary, JunctionEnrichment>();

            int id = enrichmentItems.Count;
            //for (int p = 1; p < geometricModel.Phases.Count; ++p)
            //{
            //    IPhase phase = geometricModel.Phases[p];
            //    foreach (IXFiniteElement element in phase.BoundaryElements)
            //    {
            //        // This element has already been processed when looking at another phase
            //        if (JunctionElements.ContainsKey(element)) continue;

            //        // Check if the element contains a junction point
            //        //TODO: Shouldn't the boundaries intersect?
            //        if (element.Phases.Count <= 2) continue; // Not a junction element
            //        else
            //        {
            //            var uniquePhaseSeparators = new Dictionary<int, HashSet<int>>();
            //            foreach (PhaseBoundary boundary in element.PhaseIntersections.Keys)
            //            {
            //                (IPhase minPhase, IPhase maxPhase) = FindMinMaxPhases(boundary.PositivePhase, boundary.NegativePhase);
            //                bool exists = uniquePhaseSeparators.TryGetValue(minPhase.ID, out HashSet<int> neighbors);
            //                if (!exists)
            //                {
            //                    neighbors = new HashSet<int>();
            //                    uniquePhaseSeparators[minPhase.ID] = neighbors;
            //                }
            //                neighbors.Add(maxPhase.ID);
            //            }

            //            int numUniqueSeparators = 0;
            //            foreach (HashSet<int> neighbors in uniquePhaseSeparators.Values) numUniqueSeparators += neighbors.Count;

            //            if (numUniqueSeparators <= 2) continue; // 3 or more phases, but the boundaries do not intersect
            //        }

            //        // Create a new junction enrichment
            //        // If there are n boundaries intersecting, then use n-1 junctions
            //        PhaseBoundary[] boundaries = element.PhaseIntersections.Keys.ToArray();
            //        var elementJunctions = new HashSet<JunctionEnrichment>();
            //        JunctionElements[element] = elementJunctions;
            //        for (int i = 0; i < boundaries.Length - 1; ++i) 
            //        {
            //            PhaseBoundary boundary = boundaries[i];
            //            var junction = new JunctionEnrichment(id, boundary, element.Phases);
            //            ++id;
            //            elementJunctions.Add(junction);
            //        }
            //    }
            //}
        }

        private void DefineStepEnrichments(List<EnrichmentItem> enrichmentItems)
        {
            // Keep track of identified interactions between phases, to avoid duplicate enrichments
            var uniqueEnrichments = new Dictionary<int, Dictionary<int, EnrichmentItem>>();

            foreach (IPhase phase in geometricModel.Phases.Values)
            {
                uniqueEnrichments[phase.ID] = new Dictionary<int, EnrichmentItem>();
            }

            int id = enrichmentItems.Count;
            foreach (IPhase phase in geometricModel.Phases.Values)
            {
                foreach (IPhaseBoundary boundary in phase.ExternalBoundaries)
                {
                    // It may have been processed when iterating the boundaries of the opposite phase.
                    if (boundary.StepEnrichment != null) continue;

                    // Find min/max phase IDs to uniquely identify the interaction.
                    //TODO: Why not uniquely identify the interaction based on the boundary object?
                    IPhase minPhase, maxPhase;
                    if (boundary.PositivePhase.ID < boundary.NegativePhase.ID)
                    {
                        minPhase = boundary.PositivePhase;
                        maxPhase = boundary.NegativePhase;
                    }
                    else
                    {
                        minPhase = boundary.NegativePhase;
                        maxPhase = boundary.PositivePhase;
                    }

                    // Find the existing enrichment for this phase interaction or create a new one
                    bool enrichmentsExists = 
                        uniqueEnrichments[maxPhase.ID].TryGetValue(minPhase.ID, out EnrichmentItem enrichmentItem);
                    if (!enrichmentsExists)
                    {
                        var enrichmentFunc = DefineStepEnrichment(boundary);
                        var enrichedDof = new EnrichedDof(enrichmentFunc, ThermalDof.Temperature);
                        enrichmentItem = new EnrichmentItem(id,
                            new IEnrichmentFunction[] { enrichmentFunc }, new IDofType[] { enrichedDof });
                        ++id;
                        uniqueEnrichments[maxPhase.ID][minPhase.ID] = enrichmentItem;
                        enrichmentItems.Add(enrichmentItem);
                    }

                    boundary.StepEnrichment = enrichmentItem;
                }
            }
        }

        private PhaseStepEnrichmentOLD DefineStepEnrichment(IPhaseBoundary boundary)
        {
            //TODO: This is horrible
            if (boundary.NegativePhase is DefaultPhase)
            {
                return new PhaseStepEnrichmentOLD(boundary.PositivePhase, boundary.NegativePhase);
            }
            else if (boundary.PositivePhase is DefaultPhase)
            {
                return new PhaseStepEnrichmentOLD(boundary.NegativePhase, boundary.PositivePhase);
            }
            else if (/*boundary.PositivePhase is HollowPhase &&*/ boundary.NegativePhase is ConvexPhase)
            {
                return new PhaseStepEnrichmentOLD(boundary.NegativePhase, boundary.PositivePhase);
            }
            else if (/*boundary.NegativePhase is HollowPhase &&*/ boundary.PositivePhase is ConvexPhase)
            {
                return new PhaseStepEnrichmentOLD(boundary.PositivePhase, boundary.NegativePhase);
            }
            else // Does not matter which phase will be internal/external
            {
                return new PhaseStepEnrichmentOLD(boundary.NegativePhase, boundary.PositivePhase);
            }
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

        private void EnrichNode(XNode node, IEnrichmentFunction enrichment)
        {
            if (!node.EnrichmentFuncs.ContainsKey(enrichment))
            {
                double value = enrichment.EvaluateAt(node);
                node.EnrichmentFuncs[enrichment] = value;
            }
        }

        private bool HasCorrespondingJunction(XNode node, PhaseStepEnrichmentOLD stepEnrichment) 
        {
            //TODO: Alternatively I could compare phase boundaries (discontinuities in general) instead of phases.
            foreach (IEnrichmentFunction enrichment in node.EnrichmentFuncs.Keys)
            {
                if (enrichment is JunctionEnrichment junctionEnrichment)
                {
                    // Also make sure the junction and step enrichment refer to the same phases.
                    var junctionPhases = new HashSet<IPhase>(junctionEnrichment.Phases);
                    if (junctionPhases.IsSupersetOf(stepEnrichment.Phases)) return true;
                }
            }
            return false;
        }
    }
}
