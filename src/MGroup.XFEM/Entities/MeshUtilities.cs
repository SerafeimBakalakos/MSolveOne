using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Extensions;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Entities
{
    public static class MeshUtilities
    {
        /// <summary>
        /// Finds the nodes that lie in or on the circle. It can be an empty collection. It does not search all nodes of the 
        /// mesh, but instead focuses around a provided element.
        /// </summary>
        /// <param name="circle"></param>
        /// <param name="startElement">Start by searching the nodes of this element and then move "outwards".</param>
        /// <returns>The nodes that lie in or on the circle. It can be an empty collection.</returns>
        public static HashSet<XNode> FindNodesInsideCircle(Circle2D circle, IXFiniteElement startElement)
        {
            // neighbors of the processed nodes, moving "outwards"
            var candidates = new HashSet<XNode>(startElement.Nodes); 
            var internalNodes = new HashSet<XNode>();
            var processedNodes = new HashSet<XNode>();

            bool internalNodesFound = true;
            while (internalNodesFound)
            {
                internalNodesFound = false;
                var nextCandidates = new HashSet<XNode>();
                foreach (XNode node in candidates)
                {
                    processedNodes.Add(node);
                    if (circle.SignedDistanceOf(node.Coordinates) <= 0)
                    {
                        internalNodesFound = true;
                        internalNodes.Add(node);

                        foreach (XNode other in FindNeighbors(node))
                        {
                            if (!processedNodes.Contains(other) && !candidates.Contains(other)) nextCandidates.Add(other);
                        }
                    }
                }
                candidates = nextCandidates;
            }
            return internalNodes;
        }

        public static HashSet<TElement> FindElementsIntersectedByCircle<TElement>(Circle2D circle,
            TElement startElement) where TElement : class, IXFiniteElement
        {
            //TODO: It should be rewritten. See the corresponding method for nodes.
            //TODO: It should also be tested (it is easy)
            var internalElements = new HashSet<TElement>();
            var intersectedElements = new HashSet<TElement>();

            int pos = RelativePositionOfCircleElement(circle, startElement);
            if (pos < 0) internalElements.Add(startElement);
            else if (pos == 0) intersectedElements.Add(startElement);
            else throw new ArgumentException("The provided starting element must not lie outside the circle");

            HashSet<TElement> neighbors = FindNeighbors(startElement);
            var processedElements = new HashSet<TElement>();
            processedElements.Add(startElement);
            bool allElementsExternal = false;
            while (!allElementsExternal)
            {
                allElementsExternal = true;
                var nextNeighbors = new HashSet<TElement>();
                foreach (TElement element in neighbors)
                {
                    if (!processedElements.Contains(element))
                    {
                        processedElements.Add(element);
                        pos = RelativePositionOfCircleElement(circle, element);
                        if (pos < 0)
                        {
                            internalElements.Add(element);
                            allElementsExternal = false;
                        }
                        else if (pos == 0)
                        {
                            intersectedElements.Add(element);
                            allElementsExternal = false;
                        }
                        nextNeighbors.UnionWith(FindNeighbors(element));
                    }
                }
                neighbors = nextNeighbors;
            }

            return intersectedElements;
        }

        public static HashSet<XNode> FindNeighbors(XNode node)
        {
            var neighbors = new HashSet<XNode>();
            foreach (IXFiniteElement element in node.ElementsDictionary.Values)
            {
                foreach (XNode other in element.Nodes)
                {
                    if (other != node) neighbors.Add(other);
                }
            }
            return neighbors;
        }

        public static HashSet<TElement> FindNeighbors<TElement>(TElement element) where TElement : class, IXFiniteElement
        {
            var neighbors = new HashSet<TElement>();
            foreach (XNode node in element.Nodes)
            {
                foreach (TElement other in node.ElementsDictionary.Values)
                {
                    if (other != element) neighbors.Add(other);
                }
            }
            return neighbors;
        }

        /// <summary>
        /// Positive = outside, Negative = inside, Zero = intersected
        /// </summary>
        public static int RelativePositionOfCircleElement(Circle2D circle, IElement element)
        {
            var polygon = new ConvexPolygon2D(element.Nodes.Select(n => n.Coordinates()).ToArray());
            CirclePolygonPosition pos = polygon.FindRelativePositionOfCircle(circle);
            if (pos == CirclePolygonPosition.Disjoint) return +1;
            else if (pos == CirclePolygonPosition.PolygonInsideCircle) return -1;
            else if ((pos == CirclePolygonPosition.CircleInsidePolygon) || (pos == CirclePolygonPosition.Intersecting)) return 0;
            else throw new NotImplementedException();
        }
    }
}
