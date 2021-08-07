using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.MSolve.Discretization.Mesh;
using System.Diagnostics;
using MGroup.LinearAlgebra.Vectors;

//TODO: Make sure all triangles have the same orientation. This orientation must be the same with triangles from other elements!
//      This could be done by pointing always towards a positive node. Also apply this to 2D.
//TODO: Make these intersections as smooth as the contours in ParaView
//TODO: Optimizations are possible, but may mess up readability. E.g. depending on the case, we can target specific edges that 
//      are intersected, instead of checking all of them
//TODO: For the common case, where the level set intersects the Tet4 into a triangle, there is the corner case that this triangle 
//      is extremely close to the node. In that case, it is probably safer to regard this as "Tangent". What happens if only 1 
//      or only 2 of the triangle vertices coincide with the node? 
//TODO: A lot of the same data is passed to each private utility method. Store them somewhere local and read them from there, 
//      to avoid mistakes and increase readability.
namespace MGroup.XFEM.Geometry.LSM
{
    public class LsmTet4Interaction_OLD : ILsmElementInteraction
    {
        public (RelativePositionCurveElement relativePosition, IntersectionMesh intersectionMesh) FindIntersection(
            IList<int> nodeIDs, List<double[]> nodeCoords, List<double> nodeLevelSets)
        {
            Debug.Assert(nodeCoords.Count == 4);
            Debug.Assert(nodeLevelSets.Count == 4);

            var data = new CaseData();
            data.nodeIDs = nodeIDs;
            data.nodeCoords = nodeCoords;
            data.nodeLevelSets = nodeLevelSets;
            data.intersectionMesh = new IntersectionMesh(3);

            int numZeroNodes = 0;
            int numPosNodes = 0;
            int numNegNodes = 0;
            for (int i = 0; i < 4; ++i)
            {
                if (nodeLevelSets[i] < 0) ++numNegNodes;
                else if (nodeLevelSets[i] > 0) ++numPosNodes;
                else ++numZeroNodes;
            }

            if (numZeroNodes == 0)
            {
                if ((numPosNodes == 0) || (numNegNodes == 0))
                {
                    // Disjoint
                    return (RelativePositionCurveElement.Disjoint, data.intersectionMesh);
                }
                else if ((numPosNodes == 1) || (numNegNodes == 1))
                {
                    // Intersection. 3 intersection points on edges of the single positive/negative node.
                    // The intersection mesh consists of a single triangle.
                    AddEdgeIntersections(data);
                    Debug.Assert(data.intersectionMesh.Vertices.Count == 3);
                    data.intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
                    FixTrianglesOrientation(data);
                    return (RelativePositionCurveElement.Intersecting, data.intersectionMesh);
                }
                else
                {
                    // Intersection. 4 intersection points on edges of the 2 positive nodes that connect them with the 2 
                    // negative nodes. The intersection mesh consists of 2 triangles.
                    Debug.Assert((numPosNodes == 2) && (numNegNodes == 2));
                    Process2Pos2NegCase(data);
                    FixTrianglesOrientation(data);
                    return (RelativePositionCurveElement.Intersecting, data.intersectionMesh);

                }
            }
            else if (numZeroNodes == 1)
            {
                int nodeZero = nodeLevelSets.FindIndex(phi => phi == 0);
                AddConformingNode(data, nodeZero);
                if ((numPosNodes == 0) || (numNegNodes == 0))
                {
                    // Tangent. The zero node is the only common point.
                    return (RelativePositionCurveElement.Tangent, data.intersectionMesh);
                }
                else
                {
                    // Intersection. A single positive or negative node. 2 intersection points on its edges and the zero node.
                    // The intersection mesh consists of a single triangle.
                    Debug.Assert(((numPosNodes == 1) && (numNegNodes == 2)) || ((numPosNodes == 2) && (numNegNodes == 1)));
                    AddEdgeIntersections(data);
                    Debug.Assert(data.intersectionMesh.Vertices.Count == 3);
                    data.intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
                    FixTrianglesOrientation(data);
                    return (RelativePositionCurveElement.Intersecting, data.intersectionMesh);
                }
            }
            else if (numZeroNodes == 2)
            {
                int nodeZero0 = nodeLevelSets.FindIndex(phi => phi == 0);
                int nodeZero1 = nodeLevelSets.FindLastIndex(phi => phi == 0);
                AddConformingNode(data, nodeZero0);
                AddConformingNode(data, nodeZero1);

                if ((numPosNodes == 0) || (numNegNodes == 0))
                {
                    // Tangent. The 2 zero nodes define a single common line segment, but no cell.
                    //intersectionMesh.Cells.Add((CellType.Line, new int[] { 0, 1 })); // Nope, edges are different than cells.
                    return (RelativePositionCurveElement.Tangent, data.intersectionMesh);
                }
                else
                {
                    // Intersection. 2 zero nodes and 1 intersection point on the edge connecting the positive and negative edge.
                    // The intersection mesh consists of a single triangle.
                    Debug.Assert((numPosNodes == 1) && (numPosNodes == 1));
                    AddEdgeIntersections(data);
                    Debug.Assert(data.intersectionMesh.Vertices.Count == 3);
                    data.intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
                    FixTrianglesOrientation(data);
                    return (RelativePositionCurveElement.Intersecting, data.intersectionMesh);
                }
            }
            else if (numZeroNodes == 3)
            {
                // Conforming. The intersection mesh consists of the face connecting the 3 zero nodes.
                for (int i = 0; i < 4; ++i)
                {
                    if (nodeLevelSets[i] == 0)
                    {
                        AddConformingNode(data, i);
                    }
                }
                data.intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
                FixTrianglesOrientation(data);
                return (RelativePositionCurveElement.Conforming, data.intersectionMesh);
            }
            else
            {
                // Conforming. All faces are conforming, which means that the lsm surface hugs the Tet4 and nothing else.
                Debug.Assert(numZeroNodes == 4);
                //TODO: The client should decide whether to log this msg or throw an exception
                Debug.WriteLine(
                    $"Found element that has all its faces conforming to level set surface with ID {int.MinValue}");
                for (int i = 0; i < 4; ++i)
                {
                    AddConformingNode(data, i);
                }

                // This order of vertices will cause the normals to point away from the Tet4, which coincides with the level set.
                data.intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 0, 2, 1 }));
                data.intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 3 }));
                data.intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 0, 3, 2 }));
                data.intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 1, 2, 3 }));
                return (RelativePositionCurveElement.Conforming, data.intersectionMesh);
            }
        }

        private static void Process2Pos2NegCase(CaseData data)
        {
            // Intersection. 4 intersection points on edges of the 2 positive nodes that connect them with the 2 
            // negative nodes. The intersection mesh consists of 2 triangles.

            List<double> nodeLevelSets = data.nodeLevelSets;
            List<double[]> nodeCoords = data.nodeCoords;
            var intersectionMesh = data.intersectionMesh;

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
                int[] edge = DefineIntersectedEdge(data, negNodeIdx, posNodeIdx);
                intersections.Add(intersection);
                intersectionMesh.Vertices.Add(intersection);
                intersectionMesh.IntersectedEdges.Add(edge);
            }

            List<int[]> triangles = Delauny4Points3D(intersections);
            Debug.Assert(triangles.Count == 2);
            intersectionMesh.Cells.Add((CellType.Tri3, triangles[0]));
            intersectionMesh.Cells.Add((CellType.Tri3, triangles[1]));
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

        private static int[] DefineIntersectedEdge(CaseData data, int nodeIdx0, int nodeIdx1)
        {
            IList<int> nodeIDs = data.nodeIDs;
            if (nodeIDs[nodeIdx0] < nodeIDs[nodeIdx1])
            {
                return new int[] { nodeIDs[nodeIdx0], nodeIDs[nodeIdx1] };
            }
            else
            {
                return new int[] { nodeIDs[nodeIdx1], nodeIDs[nodeIdx0] };
            }
        }

        private static void AddConformingNode(CaseData data, int nodeIdx)
        {
            data.intersectionMesh.Vertices.Add(data.nodeCoords[nodeIdx]);
            data.intersectionMesh.IntersectedEdges.Add(new int[] { data.nodeIDs[nodeIdx] });
        }

        private static void AddEdgeIntersections(CaseData data) 
        {
            List<double> nodeLevelSets = data.nodeLevelSets;
            List<double[]> nodeCoords = data.nodeCoords;
            IntersectionMesh intersectionMesh = data.intersectionMesh;
            for (int i = 0; i < 4; ++i)
            {
                for (int j = i + 1; j < 4; ++j)
                {
                    if (nodeLevelSets[i] * nodeLevelSets[j] < 0)
                    {
                        int nodeNeg, nodePos;
                        if (nodeLevelSets[i] < nodeLevelSets[j])
                        {
                            nodeNeg = i;
                            nodePos = j;
                        }
                        else
                        {
                            nodeNeg = j;
                            nodePos = i;
                        }
                        double[] intersection = Interpolate(nodeLevelSets[nodeNeg], nodeCoords[nodeNeg],
                            nodeLevelSets[nodePos], nodeCoords[nodePos]);
                        intersectionMesh.Vertices.Add(intersection);
                        intersectionMesh.IntersectedEdges.Add(DefineIntersectedEdge(data, i, j));
                    }
                }
            }
        }

        /// <summary>
        /// Fixes the oriantation of each triangle in the intersection mesh, so that its normal points towards the positive 
        /// halfspace defined by the level set surface.
        /// </summary>
        /// <param name="data"></param>
        private static void FixTrianglesOrientation(CaseData data)
        {
            List<double[]> nodeCoords = data.nodeCoords;
            List<double> nodeLevelSets = data.nodeLevelSets;
            IntersectionMesh mesh = data.intersectionMesh;

            for (int c = 0; c < mesh.Cells.Count; ++c)
            {
                // Find a normal (non-unit) of the triangle
                int[] connectivity = mesh.Cells[c].connectivity;
                var p0 = Vector.CreateFromArray(mesh.Vertices[connectivity[0]]);
                var p1 = Vector.CreateFromArray(mesh.Vertices[connectivity[1]]);
                var p2 = Vector.CreateFromArray(mesh.Vertices[connectivity[2]]);
                Vector p0p1 = p1 - p0;
                Vector p0p2 = p2 - p0;
                Vector normal = p0p1.CrossProduct(p0p2);
                //normal.ScaleIntoThis(1.0 / normal.Norm2()); // Not needed here

                // Find the node with the max distance from the triangle, by projecting onto the normal.
                // This assumes that there is at least 1 node with non-zero level set.
                // We need the max to avoid degenerate cases.
                double max = 0;
                int farthestNodeIdx = -1;
                double farthestNodeDot = double.NaN;
                for (int n = 0; n < nodeCoords.Count; ++n)
                {
                    if (nodeLevelSets[n] == 0) // These always lie on the intersection triangle
                    {
                        continue;
                    }

                    var q = Vector.CreateFromArray(nodeCoords[n]);
                    Vector p0q = q - p0;
                    double signedDistance = p0q * normal;
                    double distance = Math.Abs(signedDistance);
                    if (distance > max)
                    {
                        max = distance;
                        farthestNodeIdx = n;
                        farthestNodeDot = signedDistance;
                    }
                }
                Debug.Assert(farthestNodeIdx != -1);

                // Decide wether the normal points towards the positive halfspace. 
                bool normalPointsTowardsPositive;
                if (nodeLevelSets[farthestNodeIdx] > 0)
                {
                    // For the normal to point towards the positive halfspace, it must point towards a positive node.
                    normalPointsTowardsPositive = farthestNodeDot > 0;
                }
                else
                {
                    // For the normal to point towards the positive halfspace, it must point opposite to a negative node.
                    normalPointsTowardsPositive = farthestNodeDot < 0;
                }

                if (!normalPointsTowardsPositive) // Swap 2 vertices to flip the normal towards the positive halfspace.
                {
                    int swap = connectivity[2];
                    connectivity[2] = connectivity[1];
                    connectivity[1] = swap;
                }
            }
        }

        private static double[] Interpolate(double levelSetNeg, double[] valuesNeg, double levelSetPos, double[] valuesPos)
        {
            Debug.Assert(valuesNeg.Length == valuesPos.Length);
            if (levelSetNeg >= levelSetPos)
            {
                // This will ensure machine precision will return the same result if this method is called with the same 
                // arguments more than once.
                throw new ArgumentException("Always provide the negative level set and corresponding values first");
            }

            // The intersection point between these nodes can be found using the linear interpolation, see Sukumar 2001
            // The same interpolation can be used for coordinates or any other values.
            //
            double k = -levelSetNeg / (levelSetPos - levelSetNeg);
            var result = new double[valuesNeg.Length];
            for (int i = 0; i < valuesNeg.Length; ++i)
            {
                result[i] = valuesNeg[i] + k * (valuesPos[i] - valuesNeg[i]);
            }
            return result;
        }

        private class CaseData //TODO: Perhaps I should use instance variables and clear them at the end.
        {
            public IList<int> nodeIDs;
            public List<double[]> nodeCoords;
            public List<double> nodeLevelSets;
            public IntersectionMesh intersectionMesh;
        }
    }
}
