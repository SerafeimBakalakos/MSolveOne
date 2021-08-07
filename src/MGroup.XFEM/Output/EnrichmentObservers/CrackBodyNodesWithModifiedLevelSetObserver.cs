using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Output.EnrichmentObservers
{
    /// <summary>
    /// Tracks nodes that are enriched with crack body enrichments in this and the previous step, but their corresponding crack 
    /// body level sets were modified with respect to their values during the previous propagation step.
    /// </summary>
    public class CrackBodyNodesWithModifiedLevelSetObserver : IEnrichmentObserver
    {
        private readonly ExteriorLsmCrack crack;
        private readonly PreviousEnrichmentsObserver previousEnrichmentsObserver;
        private readonly CrackBodyNodesObserver bodyNodesObserver;

        public CrackBodyNodesWithModifiedLevelSetObserver(ExteriorLsmCrack crack, 
            PreviousEnrichmentsObserver previousEnrichmentsObserver, CrackBodyNodesObserver bodyNodesObserver)
        {
            this.crack = crack;
            this.previousEnrichmentsObserver = previousEnrichmentsObserver;
            this.bodyNodesObserver = bodyNodesObserver;
        }

        /// <summary>
        /// These may be different from the crack body level sets provided by the crack itself, since they may correspond to the
        /// previous propagation step, depending on when they are accessed.
        /// </summary>
        public HashSet<XNode> BodyNodesWithModifiedLevelSets { get; } = new HashSet<XNode>();

        public Dictionary<int, double> LevelSetsOfBodyNodes { get; private set; } = new Dictionary<int, double>();

        public void Update(IEnumerable<EnrichmentItem> allEnrichments)
        {
            Dictionary<int, double> previousBodyLevelSets = LevelSetsOfBodyNodes;
            LevelSetsOfBodyNodes = new Dictionary<int, double>();
            BodyNodesWithModifiedLevelSets.Clear();
            foreach (XNode node in bodyNodesObserver.BodyNodes)
            {
                double newLevelSet = crack.LsmGeometry.LevelSets[node.ID];
                LevelSetsOfBodyNodes[node.ID] = newLevelSet;
            }

            if (previousBodyLevelSets.Count != 0)
            {
                //bool previousBodyNodesExist = previousEnrichmentsObserver.PreviousEnrichments.TryGetValue(
                //    crack.CrackBodyEnrichment, out XNode[] nodes);
                var previousBodyNodes = new HashSet<XNode>(
                    previousEnrichmentsObserver.PreviousEnrichments[crack.CrackBodyEnrichment]);
                foreach (XNode node in bodyNodesObserver.BodyNodes)
                {
                    if (previousBodyNodes.Contains(node))
                    {
                        if (LevelSetsOfBodyNodes[node.ID] != previousBodyLevelSets[node.ID])
                        {
                            BodyNodesWithModifiedLevelSets.Add(node);
                        }
                    }
                }
            }
            
        }

        public IEnrichmentObserver[] RegisterAfterThese()
        {
            return new IEnrichmentObserver[] { previousEnrichmentsObserver, bodyNodesObserver };
        }
    }
}
