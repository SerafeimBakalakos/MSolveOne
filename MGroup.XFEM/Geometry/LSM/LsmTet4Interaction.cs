using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.MSolve.Discretization.Mesh;
using System.Diagnostics;
using MGroup.LinearAlgebra.Vectors;

//TODO: Remove duplicate code between this and the Tri3 version.
//TODO: Make these intersections as smooth as the contours in ParaView
//TODO: Optimizations are possible, but may mess up readability. E.g. depending on the case, we can target specific edges that 
//      are intersected, instead of checking all of them
//TODO: For the common case, where the level set intersects the Tet4 into a triangle, there is the corner case that this triangle 
//      is extremely close to the node. In that case, it is probably safer to regard this as "Tangent". What happens if only 1 
//      or only 2 of the triangle vertices coincide with the node? 
namespace MGroup.XFEM.Geometry.LSM
{
    public class LsmTet4Interaction : LsmCellInteractionBase
    {
        public LsmTet4Interaction(IList<int> nodeIDs, List<double[]> nodeCoords, List<double> nodeLevelSets, double tolerance)
            : base(3, 4, nodeIDs, nodeCoords, nodeLevelSets, tolerance)
        {
            Edges = new List<(int nodeIdx0, int nodeIdx1)>();
            Edges.Add((0, 1));
            Edges.Add((0, 2));
            Edges.Add((0, 3));
            Edges.Add((1, 2));
            Edges.Add((1, 3));
            Edges.Add((2, 3));

        }

        protected override List<(int nodeIdx0, int nodeIdx1)> Edges { get; }


        public override void Resolve()
        {
            (int numZeroNodes, int numPosNodes, int numNegNodes) = CountNodes();

            if (numZeroNodes == 0)
            {
                if ((numPosNodes == 0) || (numNegNodes == 0)) 
                {
                    ProcessCase0Zeros4SameSigns();
                }
                else if ((numPosNodes == 1) || (numNegNodes == 1))
                {
                    ProcessIntersectionCase(ProcessCase0Zeros3SameSigns1Different, false);
                }
                else
                {
                    Debug.Assert((numPosNodes == 2) && (numNegNodes == 2));
                    ProcessIntersectionCase(ProcessCase0Zeros2Pos2Neg, false);
                }
            }
            else if (numZeroNodes == 1)
            {
                if ((numPosNodes == 0) || (numNegNodes == 0))
                {
                    ProcessCase1Zero3SameSigns();
                }
                else
                {
                    Debug.Assert(((numPosNodes == 1) && (numNegNodes == 2)) || ((numPosNodes == 2) && (numNegNodes == 1)));
                    ProcessIntersectionCase(ProcessCase1Zero2SameSigns1Different, true);
                }
            }
            else if (numZeroNodes == 2)
            {
                if ((numPosNodes == 0) || (numNegNodes == 0))
                {
                    ProcessCase2Zeros2SameSigns();
                }
                else
                {
                    Debug.Assert((numPosNodes == 1) && (numPosNodes == 1));
                    ProcessIntersectionCase(ProcessCase2Zeros1Pos1Neg, true);
                }
            }
            else if (numZeroNodes == 3)
            {
                ProcessCase3Zeros();
            }
            else
            {
                Debug.Assert(numZeroNodes == 4);
                ProcessCase4Zeros();
            }
        }

        private void ProcessCase0Zeros4SameSigns()
        {
            // Disjoint
            Position = RelativePositionCurveElement.Disjoint;
        }

        private void ProcessCase0Zeros3SameSigns1Different()
        {
            // Intersection. 3 intersection points on edges of the single positive/negative node.
            // The intersection mesh consists of a single triangle.
            Position = RelativePositionCurveElement.Intersecting;
            AddEdgeIntersectionsToMesh();
            Debug.Assert(Mesh.Vertices.Count == 3);
            Mesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
            FixCellsOrientation();
        }

        private void ProcessCase0Zeros2Pos2Neg()
        {
            // Intersection. 4 intersection points on edges of the 2 positive nodes that connect them with the 2 
            // negative nodes. The intersection mesh consists of 2 triangles.
            Position = RelativePositionCurveElement.Intersecting;

            var posNodes = new List<int>(2);
            var negNodes = new List<int>(2);
            for (int i = 0; i < 4; ++i)
            {
                if (nodeLevelSets[i] > 0)
                {
                    posNodes.Add(i);
                }
                else
                {
                    Debug.Assert(nodeLevelSets[i] < 0);
                    negNodes.Add(i);
                }
            }

            // If P0, P1 are the positive nodes and N0, N1 the negative, then the order of intersection points in order to form
            // a simple (not self-intersecting) quadrilateral is: P0N0, N0P1, P1N1, N1P0
            int[,] pairs =
            {
                // P0N0     N0P1      P1N1      P0N1
                { 0, 0 }, { 0, 1 }, { 1, 1 }, { 1, 0 }
            };
            var intersections = new List<double[]>();
            for (int i = 0; i < 4; ++i)
            {
                int negNodeIdx = negNodes[pairs[i, 0]];
                int posNodeIdx = posNodes[pairs[i, 1]];
                double[] intersection = Interpolate(nodeLevelSets[negNodeIdx], nodeCoords[negNodeIdx],
                    nodeLevelSets[posNodeIdx], nodeCoords[posNodeIdx]);
                int[] edge = DefineIntersectedEdge(negNodeIdx, posNodeIdx);
                intersections.Add(intersection);
                Mesh.Vertices.Add(intersection);
                Mesh.IntersectedEdges.Add(edge);
            }

            List<int[]> triangles = Delauny4Points3D(intersections);
            Debug.Assert(triangles.Count == 2);
            Mesh.Cells.Add((CellType.Tri3, triangles[0]));
            Mesh.Cells.Add((CellType.Tri3, triangles[1]));
            FixCellsOrientation();
        }

        private void ProcessCase1Zero3SameSigns()
        {
            // Tangent. The zero node is the only common point.
            Position = RelativePositionCurveElement.Tangent;
            int nodeZero = nodeLevelSets.FindIndex(phi => phi == 0);
            AddConformingNodeToMesh(nodeZero);
        }

        private void ProcessCase1Zero2SameSigns1Different()
        {
            // Intersection. A single positive or negative node. 2 intersection points on its edges and the zero node.
            // The intersection mesh consists of a single triangle.
            Position = RelativePositionCurveElement.Intersecting;
            int nodeZero = nodeLevelSets.FindIndex(phi => phi == 0);
            AddConformingNodeToMesh(nodeZero);
            AddEdgeIntersectionsToMesh();
            Debug.Assert(Mesh.Vertices.Count == 3);
            Mesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
            FixCellsOrientation();
        }

        private void ProcessCase2Zeros2SameSigns()
        {
            // Tangent. The 2 zero nodes define a single common line segment, but no cell.
            Position = RelativePositionCurveElement.Tangent;
            int nodeZero0 = nodeLevelSets.FindIndex(phi => phi == 0);
            int nodeZero1 = nodeLevelSets.FindLastIndex(phi => phi == 0);
            AddConformingNodeToMesh(nodeZero0);
            AddConformingNodeToMesh(nodeZero1);
            //intersectionMesh.Cells.Add((CellType.Line, new int[] { 0, 1 })); // Nope, edges are different than cells.
        }

        private void ProcessCase2Zeros1Pos1Neg()
        {
            // Intersection. 2 zero nodes and 1 intersection point on the edge connecting the positive and negative edge.
            // The intersection mesh consists of a single triangle.
            Position = RelativePositionCurveElement.Intersecting;
            int nodeZero0 = nodeLevelSets.FindIndex(phi => phi == 0);
            int nodeZero1 = nodeLevelSets.FindLastIndex(phi => phi == 0);
            AddConformingNodeToMesh(nodeZero0);
            AddConformingNodeToMesh(nodeZero1);
            AddEdgeIntersectionsToMesh();
            Debug.Assert(Mesh.Vertices.Count == 3);
            Mesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
            FixCellsOrientation();
        }

        private void ProcessCase3Zeros()
        {
            // Conforming. The intersection mesh consists of the face connecting the 3 zero nodes.
            Position = RelativePositionCurveElement.Conforming;
            for (int i = 0; i < 4; ++i)
            {
                if (nodeLevelSets[i] == 0)
                {
                    AddConformingNodeToMesh(i);
                }
            }
            Mesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
            FixCellsOrientation();
        }

        private void ProcessCase4Zeros()
        {
            // Conforming. All faces are conforming, which means that the lsm surface hugs the Tet4 and nothing else.
            //TODO: The client should decide whether to log this msg or throw an exception
            Debug.WriteLine(
                $"Found element that has all its faces conforming to level set surface with ID {int.MinValue}");
            Position = RelativePositionCurveElement.Conforming;
            for (int i = 0; i < 4; ++i)
            {
                AddConformingNodeToMesh(i);
            }

            // This order of vertices will cause the normals to point away from the Tet4, which coincides with the level set.
            Mesh.Cells.Add((CellType.Tri3, new int[] { 0, 2, 1 }));
            Mesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 3 }));
            Mesh.Cells.Add((CellType.Tri3, new int[] { 0, 3, 2 }));
            Mesh.Cells.Add((CellType.Tri3, new int[] { 1, 2, 3 }));
        }

        protected override double[] CalcNormalOfCell(int[] cellConnectivity)
        {
            // Find a normal (non-unit) of the triangle
            var p0 = Vector.CreateFromArray(Mesh.Vertices[cellConnectivity[0]]);
            var p1 = Vector.CreateFromArray(Mesh.Vertices[cellConnectivity[1]]);
            var p2 = Vector.CreateFromArray(Mesh.Vertices[cellConnectivity[2]]);
            Vector p0p1 = p1 - p0;
            Vector p0p2 = p2 - p0;
            Vector normal = p0p1.CrossProduct(p0p2);
            //normal.ScaleIntoThis(1.0 / normal.Norm2()); // Not needed here

            return normal.RawData;
        }

        /// <summary>
        /// Finds 2 triangles out of 4 coplanar points in 3D space, which must be given in clockwise or counter-clockwise order. 
        /// These triangles observe the Delauny property: the minimum angle is the maximum possible minimum angle over all other 
        /// alternative triangulations.
        /// </summary>
        /// <param name="points">
        /// 4 coplanar points that are given in clockwise or counter-clockwise order.
        /// </param>
        /// <returns>Two arrays containing the indices into <paramref name="points"/> of the triangle vertices.</returns>
        private static List<int[]> Delauny4Points3D(List<double[]> points)
        {
            // To maximize the minimum angles, the sum of the angles opposite to the common edge must be <= pi. 
            // In this figure, this property does not hold
            //            0  
            //         /     \
            //      /           \ 
            //    1 ------------- 3
            //      \           /
            //         \     /
            //            2 

            double p0p1 = Utilities.Distance3D(points[0], points[1]);
            double p0p3 = Utilities.Distance3D(points[0], points[3]);
            double p2p1 = Utilities.Distance3D(points[2], points[1]);
            double p2p3 = Utilities.Distance3D(points[2], points[3]);
            double p1p3 = Utilities.Distance3D(points[1], points[3]);

            // Find the angles of P0, P2 using the cosine law
            double angle0 = Math.Acos((p0p1 * p0p1 + p0p3 * p0p3 - p1p3 * p1p3) / (2 * p0p1 * p0p3));
            double angle2 = Math.Acos((p2p1 * p2p1 + p2p3 * p2p3 - p1p3 * p1p3) / (2 * p2p1 * p2p3));

            var triangles = new List<int[]>(2);
            if (angle0 + angle2 <= Math.PI)
            {
                // This is the correct delauny triangulation
                triangles.Add(new int[] { 0, 1, 3 });
                triangles.Add(new int[] { 1, 2, 3 });
            }
            else
            {
                // Flip the common edge: P0P2 instead of P1P3
                triangles.Add(new int[] { 0, 1, 2 });
                triangles.Add(new int[] { 0, 2, 3 });
            }

            return triangles;
        }

        public class Factory : ILsmElementInteractionFactory
        {
            public ILsmCellInteraction CreateNewInteraction(
                IList<int> nodeIDs, List<double[]> nodeCoords, List<double> nodeLevelSets, double tolerance)
            {
                var interaction = new LsmTet4Interaction(nodeIDs, nodeCoords, nodeLevelSets, tolerance);
                interaction.Resolve();
                return interaction;
            }
        }
    }
}
