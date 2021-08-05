using System;
using System.Collections.Generic;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Phases
{
    public class ConvexPhase : IPhase
    {
        private readonly PhaseGeometryModel geometricModel;

        public ConvexPhase(int id, PhaseGeometryModel geometricModel)
        {
            this.ID = id;
            this.geometricModel = geometricModel;
        }
        public IEnumerable<IPhaseBoundary> AllBoundaries => ExternalBoundaries;

        public List<IPhaseBoundary> ExternalBoundaries { get; } = new List<IPhaseBoundary>();

        public HashSet<XNode> ContainedNodes { get; } = new HashSet<XNode>();

        public HashSet<IXMultiphaseElement> ContainedElements { get; } = new HashSet<IXMultiphaseElement>();

        public int ID { get; }

        public HashSet<IXMultiphaseElement> BoundaryElements { get; } = new HashSet<IXMultiphaseElement>();

        public HashSet<IPhase> Neighbors { get; } = new HashSet<IPhase>();

        public int MergeLevel => -1;

        public virtual bool Contains(XNode node)
        {
            foreach (ClosedPhaseBoundary boundary in ExternalBoundaries)
            {
                double distance = boundary.Geometry.SignedDistanceOf(node);
                bool sameSide = (distance > 0) && (boundary.PositivePhase == this);
                sameSide |= (distance < 0) && (boundary.NegativePhase == this);
                if (!sameSide) return false;
            }
            return true;
        }

        public virtual bool Contains(XPoint point)
        {
            foreach (ClosedPhaseBoundary boundary in ExternalBoundaries)
            {
                double distance = boundary.Geometry.SignedDistanceOf(point);
                bool sameSide = (distance > 0) && (boundary.PositivePhase == this);
                sameSide |= (distance < 0) && (boundary.NegativePhase == this);
                if (!sameSide) return false;
            }
            return true;
        }

        public void InteractWithNodes(IEnumerable<XNode> nodes)
        {
            ContainedNodes.Clear();
            foreach (XNode node in nodes)
            {
                if (Contains(node))
                {
                    ContainedNodes.Add(node);
                    node.PhaseID = this.ID;
                }
            }
        }

        public void InteractWithElements(IEnumerable<IXMultiphaseElement> elements)
        {
            //TODO: This does not necessarily provide correct results in coarse meshes.

            // Only process the elements near the contained nodes. Of course not all of them will be completely inside the phase.
            IEnumerable<IXFiniteElement> nearBoundaryElements = FindNearbyElements();
            foreach (IXMultiphaseElement element in nearBoundaryElements)
            {
                bool isInside = ContainsCompletely(element);
                if (isInside)
                {
                    ContainedElements.Add(element);
                    element.Phases.Add(this);
                }
                else
                {
                    bool isBoundary = false;
                    foreach (ClosedPhaseBoundary boundary in ExternalBoundaries)
                    {
                        // This boundary-element intersection may have already been calculated from the opposite phase. 
                        if (element.PhaseIntersections.ContainsKey(boundary))
                        {
                            isBoundary = true;
                            continue;
                        }

                        IElementDiscontinuityInteraction intersection = boundary.Geometry.Intersect(element);
                        if (intersection.RelativePosition == RelativePositionCurveElement.Intersecting)
                        {
                            element.Phases.Add(boundary.PositivePhase);
                            element.Phases.Add(boundary.NegativePhase);
                            element.PhaseIntersections[boundary] = intersection;
                            isBoundary = true;
                        }
                        else if (intersection.RelativePosition == RelativePositionCurveElement.Conforming)
                        {
                            throw new NotImplementedException();
                        }
                        else if (intersection.RelativePosition != RelativePositionCurveElement.Disjoint)
                        {
                            throw new Exception("This should not have happenned");
                        }
                    }
                    if (isBoundary) BoundaryElements.Add(element);
                }
            }
        }

        public bool UnionWith(IPhase otherPhase)
        {
            throw new InvalidOperationException();
        }

        private bool ContainsCompletely(IXFiniteElement element)
        {
            // The element is completely inside the phase if all its nodes are, since both the element and phase are convex.
            int numNodesInside = 0;
            int numNodesOutside = 0;
            foreach (XNode node in element.Nodes)
            {
                if (ContainedNodes.Contains(node)) ++numNodesInside;
                else ++numNodesOutside;
            }

            //TODO: Even if all nodes are outside, the element might still be intersected by a corner of the phase.
            //Debug.Assert(numNodesInside > 0); 

            if (numNodesOutside == 0) return true;
            else return false;

            #region this is faster, but does not take into account all cases.
            //foreach (XNode node in element.Nodes)
            //{
            //    if (!ContainedNodes.Contains(node)) return false;
            //}
            //return true;
            #endregion
        }

        private IEnumerable<IXFiniteElement> FindNearbyElements()
        {
            var nearbyElements = new HashSet<IXFiniteElement>();

            // All elements of the contained nodes. 
            foreach (XNode node in ContainedNodes)
            {
                nearbyElements.UnionWith(node.ElementsDictionary.Values);
            }

            // However an element that is intersected by just the tip of a phase corner will not be included in the above.
            // We need another layer.
            var moreNodes = new HashSet<XNode>();
            foreach (IXFiniteElement element in nearbyElements) moreNodes.UnionWith(element.Nodes);
            foreach (XNode node in moreNodes) nearbyElements.UnionWith(node.ElementsDictionary.Values);

            return nearbyElements;
        }  
    }
}
