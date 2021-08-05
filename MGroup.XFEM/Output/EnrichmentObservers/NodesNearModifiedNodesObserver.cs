using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Output.EnrichmentObservers
{
    /// <summary>
    /// Tracks nodes of the elements that contain nodes with modified enrichments, as determined by 
    /// <see cref="NodesWithModifiedEnrichmentsObserver"/>. These nose are enriched, but their enrichments are not modified.
    /// </summary>
    public class NodesNearModifiedNodesObserver : IEnrichmentObserver
    {
        private readonly NodesWithModifiedEnrichmentsObserver nodesWithModifiedEnrichmentsObserver;
        private readonly ElementsWithModifiedNodesObserver elementsWithModifiedNodesObserver;

        public NodesNearModifiedNodesObserver(NodesWithModifiedEnrichmentsObserver nodesWithModifiedEnrichmentsObserver,
            ElementsWithModifiedNodesObserver elementsWithModifiedNodesObserver)
        {
            this.nodesWithModifiedEnrichmentsObserver = nodesWithModifiedEnrichmentsObserver;
            this.elementsWithModifiedNodesObserver = elementsWithModifiedNodesObserver;
        }

        public HashSet<XNode> NearModifiedNodes { get; } = new HashSet<XNode>();

        public void Update(IEnumerable<EnrichmentItem> allEnrichments)
        {
            NearModifiedNodes.Clear();
            HashSet<XNode> modifiedNodes = nodesWithModifiedEnrichmentsObserver.ModifiedNodes;
            foreach (IXFiniteElement element in elementsWithModifiedNodesObserver.ModifiedElements)
            {
                foreach (XNode node in element.Nodes)
                {
                    if ((!modifiedNodes.Contains(node)) && (node.EnrichmentFuncs.Count > 0))
                    {
                        NearModifiedNodes.Add(node);
                    }
                }
            }
        }

        public IEnrichmentObserver[] RegisterAfterThese()
        {
            return new IEnrichmentObserver[]
            {
                nodesWithModifiedEnrichmentsObserver, elementsWithModifiedNodesObserver
            };
        }
    }
}
