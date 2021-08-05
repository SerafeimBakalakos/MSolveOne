using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Commons;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Geometry.Primitives;

//TODO: The global/local/fixed and 2D/3D hierarchies seem like a good candidate for Bridge pattern.
namespace MGroup.XFEM.Geometry.LSM.DualMesh
{
    public abstract class DualMeshLsm2DBase_OLD : IClosedGeometry
    {
        private const int dim = 2;

        protected readonly IDualMesh dualMesh;
        private readonly ValueComparer comparer;

        protected DualMeshLsm2DBase_OLD(int id, IDualMesh dualMesh)
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
            var intersectionsOfElements = new Dictionary<int, List<double[]>>();
            foreach (int fineElementID in fineElementIDs)
            {
                var intersections = IntersectFineElement(fineElementID);
                if (intersections.Count == 2)
                {
                    intersectionsOfElements[fineElementID] = intersections;
                }
            }

            // Combine the line segments into a mesh
            if (intersectionsOfElements.Count == 0)
            {
                return new NullElementDiscontinuityInteraction(this.ID, element);
            }
            var mesh = IntersectionMesh2D_OLD.CreateMesh(intersectionsOfElements);
            return new LsmElementIntersection2D(this.ID, RelativePositionCurveElement.Intersecting, element, mesh);
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

        private List<double[]> IntersectFineElement(int fineElementID)
        {
            int[] fineElementIdx = dualMesh.FineMesh.GetElementIdx(fineElementID);
            int[] fineElementNodes = dualMesh.FineMesh.GetElementConnectivity(fineElementIdx);

            var intersections = new List<double[]>();
            if (IsFineElementDisjoint(fineElementNodes)) // Check this first, since it is faster and most elements are in this category 
            {
                return intersections;
            }

            var elementGeometry = new ElementQuad4Geometry();
            (ElementEdge[] edges, _)  = elementGeometry.FindEdgesFaces(fineElementNodes);
            for (int i = 0; i < edges.Length; ++i)
            {
                int node0ID = edges[i].NodeIDs[0];
                int node1ID = edges[i].NodeIDs[1];
                double[] node0Natural = edges[i].NodesNatural[0];
                double[] node1Natural = edges[i].NodesNatural[1];
                double levelSet0 = GetLevelSet(node0ID);
                double levelSet1 = GetLevelSet(node1ID);

                if (levelSet0 * levelSet1 > 0.0) continue; // Edge is not intersected
                else if (levelSet0 * levelSet1 < 0.0) // Edge is intersected but not at its nodes
                {
                    // The intersection point between these nodes can be found using the linear interpolation, see 
                    // Sukumar 2001
                    double k = -levelSet0 / (levelSet1 - levelSet0);
                    double xi = node0Natural[0] + k * (node1Natural[0] - node0Natural[0]);
                    double eta = node0Natural[1] + k * (node1Natural[1] - node0Natural[1]);

                    AddPossiblyDuplicateIntersectionPoint(new double[] { xi, eta }, intersections);
                }
                else if ((levelSet0 == 0) && (levelSet1 == 0)) // This edge of the element conforms to the curve.
                {
                    throw new NotImplementedException();
                    //TODO: also check (DEBUG only) that all other edges are not intersected unless its is at these 2 nodes
                    //return new LsmElementIntersection2D(ID, RelativePositionCurveElement.Conforming, element,
                    //    node0Natural, node1Natural);
                }
                else if ((levelSet0 == 0) && (levelSet1 != 0)) // Curve runs through a node. Not sure if it is tangent yet.
                {
                    // Check if this node is already added. If not add it.
                    AddPossiblyDuplicateIntersectionPoint(node0Natural, intersections);
                }
                else /*if ((levelSet0 != 0) && (levelSet1 == 0))*/ // Curve runs through a node. Not sure if it is tangent yet.
                {
                    // Check if this node is already added. If not add it.
                    AddPossiblyDuplicateIntersectionPoint(node1Natural, intersections);
                }
            }

            // Convert the coordinates of the intersection points from the natural system of the fine element to the natural
            // system of the coarse element.
            for (int p = 0; p < intersections.Count; ++p)
            {
                intersections[p] = dualMesh.MapPointFineNaturalToCoarseNatural(fineElementIdx, intersections[p]);
            }
            return intersections;
        }

        private void AddPossiblyDuplicateIntersectionPoint(double[] newPoint, List<double[]> currentIntersectionPoints)
        {
            foreach (double[] point in currentIntersectionPoints)
            {
                if (PointsCoincide(point, newPoint))
                {
                    return;
                }
            }
            currentIntersectionPoints.Add(newPoint); // If this code is reached, then the new point has no duplicate.
        }

        /// <summary>
        /// Optimization for most elements. Unfortunately it may incorrectly flag an element as disjoint, e.g. if only 1 edge is 
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

        /// <summary>
        /// Optimization for most elements.
        /// </summary>
        /// <param name="element"></param>
        /// <returns></returns>
        private bool IsFineElementDisjoint(int[] fineElementNodes)
        {
            double minLevelSet = double.MaxValue;
            double maxLevelSet = double.MinValue;

            foreach (int nodeId in fineElementNodes)
            {
                double levelSet = GetLevelSet(nodeId);
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
    }
}
