//using System;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Linq;
//using System.Text;
//using MGroup.MSolve.Discretization.Mesh;
//using MGroup.MSolve.Meshes.Structured;
//using MGroup.XFEM.ElementGeometry;
//using MGroup.XFEM.Elements;
//using MGroup.XFEM.Entities;
//using MGroup.XFEM.Geometry.Primitives;
//using MGroup.XFEM.Geometry.Tolerances;

//namespace MGroup.XFEM.Geometry.LSM
//{
//    public class SimpleLsm2D_v2 : IClosedGeometry
//    {
//        private readonly CellType cellType;

//        public SimpleLsm2D_v2(int id, double[] nodalLevelSets, CellType cellType)
//        {
//            this.ID = id;
//            this.NodalLevelSets = nodalLevelSets;
//            this.cellType = cellType;
//        }

//        public SimpleLsm2D_v2(int id, IClosedManifold originalGeometry, IStructuredMesh mesh)
//        {
//            this.ID = id;
//            NodalLevelSets = new double[mesh.NumNodesTotal];
//            for (int n = 0; n < mesh.NumNodesTotal; ++n)
//            {
//                double[] node = mesh.GetNodeCoordinates(n);
//				NodalLevelSets[n] = originalGeometry.SignedDistanceOf(node);
//            }
//            this.cellType = mesh.CellType;
//        }

//        public int ID { get; }

//        public double[] NodalLevelSets { get; }

//        public IMeshTolerance MeshTolerance { get; set; } = new ArbitrarySideMeshTolerance();

//        public virtual IElementDiscontinuityInteraction Intersect(IXFiniteElement element)
//        {
//			//TODO: Use strategy pattern for the raw geometric operations.
//			if (cellType == CellType.Tri3)
//			{
//				return IntersectTri3Element(element);
//			}
//			else if (cellType == CellType.Quad4)
//			{
//				return IntersectQuad4Element(element);
//			}
//			else
//			{
//				return IntersectGeneralElement(element);
//			}
//        }

//        public double SignedDistanceOf(XNode node) => NodalLevelSets[node.ID];

//        public double SignedDistanceOf(XPoint point)
//        {
//            int[] nodes = point.Element.Nodes.Select(n => n.ID).ToArray();
//            double[] shapeFunctions = point.ShapeFunctions;
//            double result = 0;
//            for (int n = 0; n < nodes.Length; ++n)
//            {
//                result += shapeFunctions[n] * NodalLevelSets[nodes[n]];
//            }
//            return result;
//        }

//        public virtual void UnionWith(IClosedGeometry otherGeometry)
//        {
//            if (otherGeometry is SimpleLsm2D otherLsm)
//            {
//                if (this.NodalLevelSets.Length != otherLsm.NodalLevelSets.Length)
//                {
//                    throw new ArgumentException("Incompatible Level Set geometry");
//                }
//                for (int i = 0; i < this.NodalLevelSets.Length; ++i)
//                {
//                    this.NodalLevelSets[i] = Math.Min(this.NodalLevelSets[i], otherLsm.NodalLevelSets[i]);
//                }
//            }
//            else throw new ArgumentException("Incompatible Level Set geometry");
//        }

//        private IElementDiscontinuityInteraction IntersectGeneralElement(IXFiniteElement element)
//        {
//            if (IsElementDisjoint(element)) // Check this first, since it is faster and most elements are in this category 
//            {
//                return new NullElementDiscontinuityInteraction(ID, element);
//            }

//            double tol = MeshTolerance.CalcTolerance(element);
//            var intersections = new HashSet<double[]>();
//            IReadOnlyList<ElementEdge> edges = element.Edges;
//            for (int i = 0; i < edges.Count; ++i)
//            {
//                int node0ID = edges[i].NodeIDs[0];
//                int node1ID = edges[i].NodeIDs[1];
//                double[] node0Natural = edges[i].NodesNatural[0];
//                double[] node1Natural = edges[i].NodesNatural[1];
//                double levelSet0 = CalcLevelSetNearZero(node0ID, tol);
//                double levelSet1 = CalcLevelSetNearZero(node1ID, tol);

//                if (levelSet0 * levelSet1 > 0.0) continue; // Edge is not intersected
//                else if (levelSet0 * levelSet1 < 0.0) // Edge is intersected but not at its nodes
//                {
//                    // The intersection point between these nodes can be found using the linear interpolation, see 
//                    // Sukumar 2001
//                    double k = -levelSet0 / (levelSet1 - levelSet0);
//                    double xi = node0Natural[0] + k * (node1Natural[0] - node0Natural[0]);
//                    double eta = node0Natural[1] + k * (node1Natural[1] - node0Natural[1]);

//                    intersections.Add(new double[] { xi, eta });
//                }
//                else if ((levelSet0 == 0) && (levelSet1 == 0)) // Curve is tangent to the element. Edge lies on the curve.
//                {
//                    //TODO: also check (DEBUG only) that all other edges are not intersected unless its is at these 2 nodes
//                    return new LsmElementIntersection2D(ID, RelativePositionCurveElement.Conforming, element,
//                        node0Natural, node1Natural);
//                }
//                else if ((levelSet0 == 0) && (levelSet1 != 0)) // Curve runs through a node. Not sure if it is tangent yet.
//                {
//                    intersections.Add(node0Natural);
//                }
//                else /*if ((levelSet0 != 0) && (levelSet1 == 0))*/ // Curve runs through a node. Not sure if it is tangent yet.
//                {
//                    intersections.Add(node1Natural);
//                }
//            }

//            if (intersections.Count == 1) // Curve is tangent to the element at a single node
//            {
//                //TODO: Make sure the intersection point is a node (debug only)
//                return new NullElementDiscontinuityInteraction(ID, element);
//            }
//            else if (intersections.Count == 2)
//            {
//                double[][] points = intersections.ToArray();
//                return new LsmElementIntersection2D(ID, RelativePositionCurveElement.Intersecting,
//                    element, points[0], points[1]);
//            }
//            else throw new Exception("This should not have happened");
//        }

//        private IElementDiscontinuityInteraction IntersectTri3Element(IXFiniteElement element)
//        {
//            Debug.Assert(element.CellType == CellType.Tri3);

//            var nodeCoordinates = new List<double[]>();
//            var nodeLevelSets = new List<double>();
//            for (int n = 0; n < element.Nodes.Count; ++n)
//            {
//                nodeCoordinates.Add(element.Interpolation.NodalNaturalCoordinates[n]);
//                nodeLevelSets.Add(NodalLevelSets[element.Nodes[n].ID]);
//            }

//            var interactionStrategy = new LsmTri3Interaction();
//            (RelativePositionCurveElement relativePosition, IntersectionMesh2D_OLD intersectionMesh)
//                = interactionStrategy.FindIntersection(nodeCoordinates, nodeLevelSets);
//            if (relativePosition == RelativePositionCurveElement.Disjoint)
//            {
//                return new NullElementDiscontinuityInteraction(this.ID, element);
//            }
//            else if (relativePosition == RelativePositionCurveElement.Intersecting)
//            {
//                return new LsmElementIntersection2D(this.ID, relativePosition, element, intersectionMesh);
//            }
//            else if (relativePosition == RelativePositionCurveElement.Conforming)
//            {
//                return new LsmElementIntersection2D(this.ID, relativePosition, element, intersectionMesh);
//            }
//            else if (relativePosition == RelativePositionCurveElement.Tangent)
//            {
//                return new NullElementDiscontinuityInteraction(this.ID, element);
//            }
//            else
//            {
//                throw new NotImplementedException();
//            }
//        }

//		private IElementDiscontinuityInteraction IntersectQuad4Element(IXFiniteElement element)
//		{
//			throw new NotImplementedException();
//		}

//		/// <summary>
//		/// Optimization for most elements.
//		/// </summary>
//		/// <param name="element"></param>
//		/// <returns></returns>
//		private bool IsElementDisjoint(IXFiniteElement element)
//        {
//            double minLevelSet = double.MaxValue;
//            double maxLevelSet = double.MinValue;

//            foreach (XNode node in element.Nodes)
//            {
//                double levelSet = NodalLevelSets[node.ID];
//                if (levelSet < minLevelSet) minLevelSet = levelSet;
//                if (levelSet > maxLevelSet) maxLevelSet = levelSet;
//            }

//            if (minLevelSet * maxLevelSet > 0.0) return true;
//            else return false;
//        }

//        private double CalcLevelSetNearZero(int nodeID, double zeroTolerance)
//        {
//            double levelSet = NodalLevelSets[nodeID];
//            if (Math.Abs(levelSet) <= zeroTolerance) return 0.0;
//            else return levelSet;
//        }
//    }
//}
