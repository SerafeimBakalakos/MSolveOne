using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Output.EnrichmentObservers
{
    /// <summary>
    /// Tracks nodes with different enrichments or different values of enrichment functions between two consecutive
    /// crack propagation steps. These changes may result from application or removal of crack tip enrichments, application of 
    /// crack body enrichments or change in the values of crack body enrichments or level sets, due to the crack curving.
    /// </summary>
    public class NodesWithModifiedEnrichmentsObserver : IEnrichmentObserver
    {
        private readonly CrackBodyNodesWithModifiedLevelSetObserver bodyNodesWithModifiedLevelSetObserver;
        private readonly NewCrackBodyNodesObserver newBodyNodesObserver;
        private readonly NewCrackTipNodesObserver newTipNodesObserver;
        private readonly PreviousCrackTipNodesObserver oldTipNodesObserver;

        public NodesWithModifiedEnrichmentsObserver(
            NewCrackTipNodesObserver newTipNodesObserver, 
            PreviousCrackTipNodesObserver oldTipNodesObserver,
            NewCrackBodyNodesObserver newBodyNodesObserver,
            CrackBodyNodesWithModifiedLevelSetObserver bodyNodesWithModifiedLevelSetObserver)
        {
            this.newTipNodesObserver = newTipNodesObserver;
            this.oldTipNodesObserver = oldTipNodesObserver;
            this.newBodyNodesObserver = newBodyNodesObserver;
            this.bodyNodesWithModifiedLevelSetObserver = bodyNodesWithModifiedLevelSetObserver;
        }

        public HashSet<XNode> ModifiedNodes { get; } = new HashSet<XNode>();

        public void Update(IEnumerable<EnrichmentItem> allEnrichments)
        {
            ModifiedNodes.Clear();
            ModifiedNodes.UnionWith(newTipNodesObserver.TipNodes);
            ModifiedNodes.UnionWith(oldTipNodesObserver.PreviousTipNodes);
            ModifiedNodes.UnionWith(newBodyNodesObserver.NewBodyNodes);
            ModifiedNodes.UnionWith(bodyNodesWithModifiedLevelSetObserver.BodyNodesWithModifiedLevelSets);
        }

        public IEnrichmentObserver[] RegisterAfterThese()
        {
            return new IEnrichmentObserver[] 
            { 
                newTipNodesObserver, oldTipNodesObserver, newBodyNodesObserver, bodyNodesWithModifiedLevelSetObserver
            };
        }
    }
}
