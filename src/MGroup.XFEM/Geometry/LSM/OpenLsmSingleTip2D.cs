using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

//TODO: Nodes define the state of this object, don't they? Therefore shouldn't they be stored in it, instead of being passed into
//      this class's methods from client code? The problem with that are state changes, e.g. nodes being added removed to 
//      represent the geometry or the model itself. Another problem is distributed environments, where it would be cumbersome
//      for this class to manage the decomposition. Perhaps it is necessary though.
namespace MGroup.XFEM.Geometry.LSM
{
    public class OpenLsmSingleTip2D : ISingleTipLsmGeometry
    {
        private readonly IOpenLevelSetUpdater levelSetUpdater;

        public OpenLsmSingleTip2D(int id)
        {
            this.ID = id;
            this.levelSetUpdater = new LevelSetUpdater2DStolarska();
        }

        public int ID { get; }

        Dictionary<int, double> ILsmGeometry.LevelSets => LevelSetsBody;
        public Dictionary<int, double> LevelSetsBody { get; } = new Dictionary<int, double>();
        public Dictionary<int, double> LevelSetsTip { get; } = new Dictionary<int, double>();

        public double[] Tip { get; private set; }

        public FrontCoordinateSystemExplicit FrontSystem { get; private set; }

        public void Initialize(IEnumerable<XNode> nodes, PolyLine2D initialCurve)
        {
            //TODO: The tangent stuff should be done by the initial curve.
            int tipVertexIdx = initialCurve.Vertices.Count - 1;
            double[] tip = initialCurve.Vertices[tipVertexIdx];
            double tangentX = tip[0] - initialCurve.Vertices[tipVertexIdx - 1][0];
            double tangentY = tip[1] - initialCurve.Vertices[tipVertexIdx - 1][1];
            double length = Math.Sqrt(tangentX * tangentX + tangentY * tangentY);
            double tangentSlope = Math.Atan2(tangentY, tangentX);
            tangentX /= length;
            tangentY /= length;

            this.Tip = tip;
            this.FrontSystem = new FrontCoordinateSystemExplicit(tip, tangentSlope);

            foreach (XNode node in nodes)
            {
                LevelSetsBody[node.ID] = initialCurve.SignedDistanceOf(node.Coordinates);
                LevelSetsTip[node.ID] = (node.Coordinates[0] - tip[0]) * tangentX + (node.Coordinates[1] - tip[1]) * tangentY;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <remarks>Stolarska has proposed a simpler approach for the characterization (tip element, cut or standard), but that  
        /// is not always correct.</remarks>
        /// <param name="element"></param>
        public IElementDiscontinuityInteraction Intersect(IXFiniteElement element)
        {
            (Dictionary<int, double> nodalBodyLevelSets, Dictionary<int, double> nodalTipLevelSets) = 
                FindLevelSetsOfElementNodes(element);

            // Check this first, since it is faster and most elements belong to this category 
            if (IsElementDisjoint(element, nodalBodyLevelSets, nodalTipLevelSets)) 
            {
                return new NullElementDiscontinuityInteraction(ID, element);
            }

            (bool conforming, List<IntersectionPoint> intersections) = 
                FindIntersectionPoints(element, nodalBodyLevelSets, nodalTipLevelSets);

            if (intersections.Count == 1) // The only common point is a node 
            {
                if (intersections[0].TipLevelSet > 0)
                {
                    // The node lies on the curve's extension 
                    return new NullElementDiscontinuityInteraction(this.ID, element);
                }
                else if (intersections[0].TipLevelSet < 0)
                {
                    // The node lies on the curve before the tip 
                    return new OpenLsmElementIntersection2D(this.ID, element, RelativePositionCurveElement.Tangent, false,
                        new double[][] { intersections[0].CoordinatesNatural });
                }
                else
                {
                    // The node coincides with the tip
                    return new OpenLsmElementIntersection2D(this.ID, element, RelativePositionCurveElement.Tangent, true,
                        new double[][] { intersections[0].CoordinatesNatural });
                }
            }
            else // 2 intersection points
            {
                // Sort intersection points w.r.t. psi
                IntersectionPoint point0, point1;
                if (intersections[0].TipLevelSet < intersections[1].TipLevelSet)
                {
                    point0 = intersections[0];
                    point1 = intersections[1];
                }
                else
                {
                    point0 = intersections[1];
                    point1 = intersections[0];
                }

                // Check if the intersection points lie on the curve or its extension 
                if (point0.TipLevelSet > 0)
                {
                    // Both points lie on the extension
                    return new NullElementDiscontinuityInteraction(this.ID, element);
                }
                else if (point1.TipLevelSet < 0)
                {
                    // Both points lie on the curve before the tip
                    var pos = conforming ? RelativePositionCurveElement.Conforming : RelativePositionCurveElement.Intersecting;
                    return new OpenLsmElementIntersection2D(this.ID, element, pos, false,
                        new double[][] { point0.CoordinatesNatural, point1.CoordinatesNatural });
                }
                else
                {
                    // The tip lies in the element or on its boundary. First find its coordinates using linear interpolation.
                    double k = -point0.TipLevelSet / (point1.TipLevelSet - point0.TipLevelSet);
                    if (k < 0) k = 0; // Make sure it does not end up outside the element, due to limited precision.
                    if (k > 1) k = 1;
                    var tipCoordsNatural = new double[2];
                    for (int d = 0; d < 2; ++d)
                    {
                        tipCoordsNatural[d] = (1 - k) * point0.CoordinatesNatural[d] + k * point1.CoordinatesNatural[d];
                    }

                    if (point0.TipLevelSet < 0)
                    {
                        // A segment of the curve is inside the element or on its boundary
                        var pos = conforming ? RelativePositionCurveElement.Conforming : RelativePositionCurveElement.Intersecting;
                        return new OpenLsmElementIntersection2D(this.ID, element, pos, true,
                            new double[][] { point0.CoordinatesNatural, tipCoordsNatural });
                    }
                    else
                    {
                        // The only common point is the curve tip, which lies on the element's boundary
                        return new OpenLsmElementIntersection2D(this.ID, element, RelativePositionCurveElement.Tangent, true,
                            new double[][] { tipCoordsNatural });
                    }
                }
            }
        }

        public double SignedDistanceOf(XNode node) => LevelSetsBody[node.ID];

        public double SignedDistanceOf(XPoint point)
        {
            IReadOnlyList<XNode> nodes = point.Element.Nodes;
            double signedDistance = 0.0;
            for (int n = 0; n < nodes.Count; ++n)
            {
                signedDistance += point.ShapeFunctions[n] * LevelSetsBody[nodes[n].ID];
            }
            return signedDistance;
        }

        public double[] SignedDistanceGradientThrough(XPoint point)
        {
            IReadOnlyList<XNode> nodes = point.Element.Nodes;
            double gradientX = 0.0;
            double gradientY = 0.0;
            for (int n = 0; n < nodes.Count; ++n)
            {
                double dNdx = point.ShapeFunctionDerivativesGlobal[n, 0];
                double dNdy = point.ShapeFunctionDerivativesGlobal[n, 1];

                double levelSet = LevelSetsBody[nodes[n].ID];
                gradientX += dNdx * levelSet;
                gradientY += dNdy * levelSet;
            }
            return new double[] { gradientX, gradientY };
        }

        public void Update(IEnumerable<XNode> nodes, double localGrowthAngle, double growthLength)
        {
            double globalGrowthAngle = Utilities.WrapAngle(localGrowthAngle + FrontSystem.RotationAngle);
            double dx = growthLength * Math.Cos(globalGrowthAngle);
            double dy = growthLength * Math.Sin(globalGrowthAngle);
            var oldTip = this.Tip;
            var newTip = new double[] { oldTip[0] + dx, oldTip[1] + dy };
            this.Tip = newTip;
            this.FrontSystem = new FrontCoordinateSystemExplicit(newTip, globalGrowthAngle);

            levelSetUpdater.Update(oldTip, newTip, nodes, LevelSetsBody, LevelSetsTip);
        }

        private static (bool conforming, List<IntersectionPoint> intersections) FindIntersectionPoints(IXFiniteElement element, 
            Dictionary<int, double> bodyLevelSets, Dictionary<int, double> tipLevelSets)
        {
            var intersections = new List<IntersectionPoint>(2);

            // First process nodes, since they will appear multiple times when going over edges
            IReadOnlyList<double[]> nodesNatural = element.Interpolation.NodalNaturalCoordinates;
            for (int n = 0; n < element.Nodes.Count; ++n)
            {
                XNode node = element.Nodes[n];
                if (bodyLevelSets[node.ID] == 0)
                {
                    intersections.Add(new IntersectionPoint() 
                    { 
                        CoordinatesNatural = nodesNatural[n], 
                        TipLevelSet = tipLevelSets[node.ID] 
                    });
                }
            }

            // Check if each edge is intersected by the curve
            IReadOnlyList<ElementEdge> edges = element.Edges;
            for (int i = 0; i < edges.Count; ++i)
            {
                int node0ID = edges[i].NodeIDs[0];
                int node1ID = edges[i].NodeIDs[1];
                double[] node0Natural = edges[i].NodesNatural[0];
                double[] node1Natural = edges[i].NodesNatural[1];
                double phi0 = bodyLevelSets[node0ID];
                double phi1 = bodyLevelSets[node1ID];

                if (phi0 * phi1 > 0.0) continue; // Edge is not intersected
                else if (phi0 * phi1 < 0.0) // Edge is intersected but not at its nodes
                {
                    double psi0 = tipLevelSets[node0ID];
                    double psi1 = tipLevelSets[node1ID];

                    // The intersection point between these nodes can be found using the linear interpolation, see 
                    // Sukumar 2001
                    double k = -phi0 / (phi1 - phi0);
                    double xi = node0Natural[0] + k * (node1Natural[0] - node0Natural[0]);
                    double eta = node0Natural[1] + k * (node1Natural[1] - node0Natural[1]);
                    double psi = psi0 + k * (psi1 - psi0);

                    intersections.Add(new IntersectionPoint()
                    {
                        CoordinatesNatural = new double[] { xi, eta },
                        TipLevelSet = psi
                    });
                }
                else if ((phi0 == 0) && (phi1 == 0)) // Element edge conforms to the curve
                {
                    // If no gross errors were made, these 2 nodes should be the only 2 intersection points
                    //TODO: also check (DEBUG only) that all other edges are not intersected unless its is at these 2 nodes
                    Debug.Assert(intersections.Count == 2);
                    return (true, intersections);
                }
            }
            Debug.Assert((intersections.Count == 1) || (intersections.Count == 2));
            return (false, intersections);
        }

        private (Dictionary<int, double> bodyLevelSets, Dictionary<int, double> tipLevelSets) FindLevelSetsOfElementNodes(
            IXFiniteElement element)
        {
            int numNodes = element.Nodes.Count;
            var bodyLevelSets = new Dictionary<int, double>(numNodes);
            var tipLevelSets = new Dictionary<int, double>(numNodes);
            for (int n = 0; n < numNodes; ++n)
            {
                int nodeID = element.Nodes[n].ID;
                bodyLevelSets[nodeID] = LevelSetsBody[nodeID];
                tipLevelSets[nodeID] = LevelSetsTip[nodeID];
            }
            return (bodyLevelSets, tipLevelSets);
        }

        /// <summary>
        /// Optimization for most elements. It is possible for this method to return false, even if the element is disjoint.
        /// </summary>
        /// <param name="element"></param>
        private static bool IsElementDisjoint(IXFiniteElement element, 
            Dictionary<int, double> nodalBodyLevelSets, Dictionary<int, double> nodalTipLevelSets)
        {
            double minBodyLS = double.MaxValue;
            double maxBodyLS = double.MinValue;
            double minTipLS = double.MaxValue;

            foreach (XNode node in element.Nodes)
            {
                double bodyLS = nodalBodyLevelSets[node.ID];
                if (bodyLS < minBodyLS) minBodyLS = bodyLS;
                if (bodyLS > maxBodyLS) maxBodyLS = bodyLS;

                double tipLS = nodalTipLevelSets[node.ID];
                if (tipLS < minTipLS) minTipLS = tipLS;
            }

            if (minBodyLS * maxBodyLS > 0.0) return true;
            else if (minTipLS > 0.0) return true;
            else return false;
        }

        private class IntersectionPoint
        {
            public double[] CoordinatesNatural { get; set; }

            public double TipLevelSet { get; set; }
        }
    }
}
