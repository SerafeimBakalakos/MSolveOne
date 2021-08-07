using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Commons;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Interpolation;

namespace MGroup.XFEM.Geometry.LSM.DualMesh
{
    public abstract class DualMeshLsm3DBase_OLD : IClosedGeometry
    {
        private const int dim = 3;

        protected readonly DualCartesianMesh3D dualMesh;
        private readonly ValueComparer comparer;

        protected DualMeshLsm3DBase_OLD(int id, DualCartesianMesh3D dualMesh)
        {
            this.dualMesh = dualMesh;
            this.ID = id;
            this.comparer = new ValueComparer(1E-6);
        }

        public int ID { get; }

        //TODO: How can I check and what to do if the intersection mesh or part of it conforms to the element edges?
        public IElementDiscontinuityInteraction Intersect(IXFiniteElement element)
        {
            // WARNING: This optimization must be avoided. Coarse elements may be flagged as disjoint incorrectly .
            //if (IsCoarseElementDisjoint(element)) return new NullElementDiscontinuityInteraction(this.ID, element);

            int[] fineElementIDs = dualMesh.MapElementCoarseToFine(element.ID);
            var intersectionsOfElements = new Dictionary<int, IntersectionMesh3D_OLD>();
            foreach (int fineElementID in fineElementIDs)
            {
                int[] fineElementIdx = dualMesh.FineMesh.GetElementIdx(fineElementID);

                int[] fineElementNodes = dualMesh.FineMesh.GetElementConnectivity(fineElementIdx);
                RelativePositionCurveElement position = FindRelativePosition(fineElementNodes);
                if ((position == RelativePositionCurveElement.Disjoint) || (position == RelativePositionCurveElement.Tangent))
                {
                    // Do nothing
                }
                else if (position == RelativePositionCurveElement.Intersecting)
                {
                    IntersectionMesh3D_OLD intersectionMesh = FindInteractionIntersecting(fineElementIdx, fineElementNodes);
                    intersectionsOfElements[fineElementID] = intersectionMesh;
                }
                else if (position == RelativePositionCurveElement.Conforming)
                {
                    throw new NotImplementedException();
                }
                else throw new NotImplementedException();
            }
            if (intersectionsOfElements.Count == 0)
            {
                //TODO: This needs adjustment to take into account conforming elements
                return new NullElementDiscontinuityInteraction(this.ID, element);
            }

            // Combine the line segments into a mesh
            if (intersectionsOfElements.Count == 0)
            {
                return new NullElementDiscontinuityInteraction(this.ID, element);
            }
            var jointIntersectionMesh = IntersectionMesh3D_OLD.JoinMeshes(intersectionsOfElements);
            return new LsmElementIntersection3D(ID, RelativePositionCurveElement.Intersecting, element, jointIntersectionMesh);
        }

        public double SignedDistanceOf(XNode node) => GetLevelSet(dualMesh.MapNodeIDCoarseToFine(node.ID));

        public double SignedDistanceOf(XPoint point)
        {
            int coarseElementID = point.Element.ID;
            double[] coarseNaturalCoords = point.Coordinates[CoordinateSystem.ElementNatural];
            DualMeshPoint dualMeshPoint = dualMesh.CalcShapeFunctions(coarseElementID, coarseNaturalCoords);
            double[] shapeFunctions = dualMeshPoint.FineShapeFunctions;
            int[] fineNodes = dualMesh.FineMesh.GetElementConnectivity(dualMeshPoint.FineElementIdx);

            double result = 0;
            for (int n = 0; n < fineNodes.Length; ++n)
            {
                result += shapeFunctions[n] * GetLevelSet(fineNodes[n]);
            }
            return result;
        }

        public abstract void UnionWith(IClosedGeometry otherGeometry);

        protected abstract double GetLevelSet(int fineNodeID);

        private IntersectionMesh3D_OLD FindInteractionIntersecting(int[] fineElementIdx, int[] fineNodeIDs)
        {
            var elementGeometry = new ElementHexa8Geometry();
            (ElementEdge[] edges, ElementFace[] allFaces) = elementGeometry.FindEdgesFaces(fineNodeIDs);
            IReadOnlyList<double[]> nodesNatural = InterpolationHexa8.UniqueInstance.NodalNaturalCoordinates;

            var intersectionPoints = new Dictionary<double[], HashSet<ElementFace>>();

            // Find any nodes that may lie on the LSM geometry
            var comparer = new ValueComparer(1E-7);
            for (int n = 0; n < fineNodeIDs.Length; ++n)
            {
                int nodeID = fineNodeIDs[n];
                if (comparer.AreEqual(0, GetLevelSet(nodeID)))
                {
                    HashSet<ElementFace> facesOfNode = ElementFace.FindFacesOfNode(nodeID, allFaces);
                    double[] intersection = nodesNatural[n];
                    if (!PointExistsAlready(intersection, intersectionPoints.Keys))
                    {
                        intersectionPoints.Add(nodesNatural[n], facesOfNode);
                    }
                }
            }

            // Find intersection points that lie on element edges, excluding nodes
            foreach (ElementEdge edge in edges)
            {
                double[] intersection = IntersectEdgeExcludingNodes(edge);
                if (intersection != null)
                {
                    if (!PointExistsAlready(intersection, intersectionPoints.Keys))
                    {
                        HashSet<ElementFace> facesOfEdge = edge.FindFacesOfEdge(allFaces);
                        intersectionPoints.Add(intersection, facesOfEdge);
                    }
                }
            }

            // Convert the coordinates of the intersection points from the natural system of the fine element to the natural
            // system of the FEM element.
            var intersectionPointsCoarse = new Dictionary<double[], HashSet<ElementFace>>();
            foreach (var pair in intersectionPoints)
            {
                double[] pointFine = pair.Key;
                double[] pointCoarse = dualMesh.MapPointFineNaturalToCoarseNatural(fineElementIdx, pointFine);
                intersectionPointsCoarse[pointCoarse] = pair.Value;
            }

            // Create mesh
            return IntersectionMesh3D_OLD.CreateMultiCellMesh3D(intersectionPointsCoarse);
        }

        private RelativePositionCurveElement FindRelativePosition(int[] fineElementNodes)
        {
            int numPositiveNodes = 0;
            int numNegativeNodes = 0;
            int numZeroNodes = 0;
            foreach (int nodeID in fineElementNodes)
            {
                double levelSet = GetLevelSet(nodeID);
                if (comparer.AreEqual(0, levelSet)) ++numZeroNodes;
                else if (levelSet > 0) ++numPositiveNodes;
                else /*if (levelSet < 0)*/ ++numNegativeNodes;
            }

            if ((numPositiveNodes == fineElementNodes.Length) || (numNegativeNodes == fineElementNodes.Length))
            {
                return RelativePositionCurveElement.Disjoint;
            }
            else if ((numPositiveNodes > 0) && (numNegativeNodes > 0))
            {
                return RelativePositionCurveElement.Intersecting;
            }
            else if (numZeroNodes < 3)
            {
                // The surface is assumed to be a plane. In rare cases, it can go through 1 node or 1 edge. 
                // Even then, no surface segment can be defined.
                //TODO: Is the assumption that surface == plane correct? 
                return RelativePositionCurveElement.Tangent;
            }
            else
            {
                // One of the element's faces conforms to the surface.
                //TODO: Assert that all zero nodes do indeed belong to the same face
                return RelativePositionCurveElement.Conforming;
            }
        }

        private double[] IntersectEdgeExcludingNodes(ElementEdge edge)
        {
            double levelSet0 = GetLevelSet(edge.NodeIDs[0]);
            double levelSet1 = GetLevelSet(edge.NodeIDs[1]);
            double[] node0 = edge.NodesNatural[0];
            double[] node1 = edge.NodesNatural[1];

            if (levelSet0 * levelSet1 < 0.0) // Edge is intersected but not at its nodes
            {
                // The intersection point between these nodes can be found using the linear interpolation, see 
                // Sukumar 2001
                double k = -levelSet0 / (levelSet1 - levelSet0);
                var intersection = new double[3];
                for (int d = 0; d < 3; ++d)
                {
                    intersection[d] = node0[d] + k * (node1[d] - node0[d]);
                }
                return intersection;
            }
            else return null;
        }

        /// <summary>
        /// Optimization for most elements. Unfortunately it may incorrectly flag an element as disjoint, e.g. if only 1 face is 
        /// intersected.
        /// </summary>
        /// <param name="element"></param>
        /// <returns></returns>
        private bool IsCoarseElementDisjoint(IXFiniteElement element)
        {
            double minLevelSet = double.MaxValue;
            double maxLevelSet = double.MinValue;

            foreach (XNode node in element.Nodes)
            {
                int fineNodeID = dualMesh.MapNodeIDCoarseToFine(node.ID);
                double levelSet = GetLevelSet(fineNodeID);
                if (levelSet < minLevelSet) minLevelSet = levelSet;
                if (levelSet > maxLevelSet) maxLevelSet = levelSet;
            }

            if (minLevelSet * maxLevelSet > 0.0) return true;
            else return false;
        }

        private bool PointsCoincide(double[] point0, double[] point1)
        {
            //TODO: Possibly add some tolerance
            for (int d = 0; d < dim; ++d)
            {
                if (!comparer.AreEqual(point0[d], point1[d]))
                {
                    return false;
                }
            }
            return true;
        }

        private bool PointExistsAlready(double[] newPoint, IEnumerable<double[]> currentIntersectionPoints)
        {
            foreach (double[] point in currentIntersectionPoints)
            {
                if (PointsCoincide(point, newPoint))
                {
                    return true;
                }
            }
            return false;
        }
    }
}
