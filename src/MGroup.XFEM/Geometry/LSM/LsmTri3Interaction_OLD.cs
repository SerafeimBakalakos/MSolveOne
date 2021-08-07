using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using System.Diagnostics;

namespace MGroup.XFEM.Geometry.LSM
{
    public class LsmTri3Interaction_OLD : ILsmElementInteraction
    {
        public (RelativePositionCurveElement relativePosition, IntersectionMesh intersectionMesh) 
            FindIntersection(IList<int> nodeIDs, List<double[]> nodeCoords, List<double> nodeLevelSets)
        {
            Debug.Assert(nodeCoords.Count == 3);
            Debug.Assert(nodeLevelSets.Count == 3);

            int numZeroNodes = 0;
            int numPosNodes = 0;
            int numNegNodes = 0;
            for (int i = 0; i < 3; ++i)
            {
                if (nodeLevelSets[i] < 0) ++numNegNodes;
                else if (nodeLevelSets[i] > 0) ++numPosNodes;
                else ++numZeroNodes;
            }

            var intersectionMesh = new IntersectionMesh(2);
            if (numZeroNodes == 0)
            {
                if ((numPosNodes == 0) || (numNegNodes == 0)) // Disjoint
                {
                    return (RelativePositionCurveElement.Disjoint, intersectionMesh);
                }
                else // 2 intersection points
                {
                    for (int i = 0; i < 3; ++i)
                    {
                        int j = (i + 1) % 3;
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
                            intersectionMesh.IntersectedEdges.Add(DefineIntersectedEdge(nodeIDs, i, j));
                        }
                    }
                    Debug.Assert(intersectionMesh.Vertices.Count == 2);

                    // Find the orientation of the segment, such that its normal points towards the positive halfspace
                    double[] pointA = intersectionMesh.Vertices[0];
                    double[] pointB = intersectionMesh.Vertices[1];
                    double[] pointPos = nodeCoords[FindIdxOfMostPositiveNode(nodeLevelSets)];
                    int[] cell = IsNormalTowardsPositive(pointA, pointB, pointPos) ? new int[] { 0, 1 } : new int[] { 1, 0 };
                    intersectionMesh.Cells.Add((CellType.Line2, cell));

                    return (RelativePositionCurveElement.Intersecting, intersectionMesh);
                }
            }
            else if (numZeroNodes == 1)
            {
                int nodeZero = nodeLevelSets.FindIndex(phi => phi == 0);
                intersectionMesh.Vertices.Add(nodeCoords[nodeZero]);
                intersectionMesh.IntersectedEdges.Add(new int[] { nodeIDs[nodeZero] });
                if ((numPosNodes == 0) || (numNegNodes == 0)) // Tangent (only 1 common point)
                {
                    return (RelativePositionCurveElement.Tangent, intersectionMesh);
                }
                else // 1 intersection point and 1 node
                {
                    int nodeNeg = nodeLevelSets.FindIndex(phi => phi < 0);
                    int nodePos = nodeLevelSets.FindIndex(phi => phi > 0);
                    double[] intersection = Interpolate(nodeLevelSets[nodeNeg], nodeCoords[nodeNeg],
                        nodeLevelSets[nodePos], nodeCoords[nodePos]);
                    intersectionMesh.Vertices.Add(intersection);
                    intersectionMesh.IntersectedEdges.Add(DefineIntersectedEdge(nodeIDs, nodeNeg, nodePos));

                    // Find the orientation of the segment, such that its normal points towards the positive halfspace
                    double[] pointA = intersectionMesh.Vertices[0];
                    double[] pointB = intersectionMesh.Vertices[1];
                    double[] pointPos = nodeCoords[nodePos];
                    int[] cell = IsNormalTowardsPositive(pointA, pointB, pointPos) ? new int[] { 0, 1 } : new int[] { 1, 0 };
                    intersectionMesh.Cells.Add((CellType.Line2, cell));

                    return (RelativePositionCurveElement.Intersecting, intersectionMesh);
                }
            }
            else if (numZeroNodes == 2) // 1 conforming edge
            {
                int node0 = nodeLevelSets.FindIndex(phi => phi == 0);
                int node1 = nodeLevelSets.FindIndex(phi => phi == 0);
                intersectionMesh.Vertices.Add(nodeCoords[node0]);
                intersectionMesh.Vertices.Add(nodeCoords[node1]);
                intersectionMesh.IntersectedEdges.Add(DefineIntersectedEdge(nodeIDs, node0, node1));

                // Find the orientation of the segment, such that its normal points towards the positive halfspace
                double[] pointA = intersectionMesh.Vertices[0];
                double[] pointB = intersectionMesh.Vertices[1];
                int node2 = nodeLevelSets.FindIndex(phi => phi != 0);
                double[] pointPos = nodeCoords[node2]; // assume it is positive and correct later
                bool isNormalPositive = IsNormalTowardsPositive(pointA, pointB, pointPos);
                if (nodeLevelSets[node2] < 0) isNormalPositive = !isNormalPositive; // now correct the original assumption
                int[] cell = isNormalPositive ? new int[] { 0, 1 } : new int[] { 1, 0 };
                intersectionMesh.Cells.Add((CellType.Line2, cell));

                return (RelativePositionCurveElement.Conforming, intersectionMesh);
            }
            else // 3 conforming edges
            {
                Debug.Assert(numZeroNodes == 3);
                //TODO: The client should decide whether to log this msg or throw an exception
                Debug.WriteLine(
                    $"Found element that has all its edges conforming to level set curve with ID {int.MinValue}." +
                    $" This usually indicates an error. It may also cause problems if the triangle nodes are not given in" +
                    $" counter-clockwise order.");
                for (int i = 0; i < 3; ++i)
                {
                    int j = (i + 1) % 3;
                    intersectionMesh.Vertices.Add(nodeCoords[i]);
                    intersectionMesh.IntersectedEdges.Add(DefineIntersectedEdge(nodeIDs, i, j));

                    // We assume that i) the level set encircles this element and intersects no other, ii) the interior is 
                    // negative and the exterior positive, iii) the triangle's nodes are in counter-clockwise order. 
                    // Thus if we traverse each edge is i+1 -> i, then the normal will point outside, meaning towards positive.
                    intersectionMesh.Cells.Add((CellType.Line2, new int[] { j, i })); 
                }
                return (RelativePositionCurveElement.Conforming, intersectionMesh);
            }
        }

        /// <summary>
        /// Returns -1, if there is no positive node
        /// </summary>
        /// <param name="nodeLevelSets"></param>
        /// <returns></returns>
        private static int FindIdxOfMostPositiveNode(List<double> nodeLevelSets)
        {
            double max = 0;
            int target = -1;
            for (int n = 0; n < nodeLevelSets.Count; ++n)
            {
                if (nodeLevelSets[n] > max)
                {
                    max = nodeLevelSets[n];
                    target = n;
                }
            }
            if (max > 0) return target;
            else return -1;
        }

        /// <summary>
        /// </summary>
        /// <param name="pointA"></param>
        /// <param name="pointB"></param>
        /// <param name="pointPositive">A point in the positive halfspace</param>
        private static bool IsNormalTowardsPositive(double[] pointA, double[] pointB, double[] pointPositive)
        {
            double[] normal = { -(pointB[1] - pointA[1]), pointB[0] - pointA[0] }; // normal to AB, pi/2 counter-clockwise
            double[] vectorAP = { pointPositive[0] - pointA[0], pointPositive[1] - pointA[1] };
            double dot = normal[0] * vectorAP[0] + normal[1] * vectorAP[1];
            if (dot > 0) return true;
            else if (dot < 0) return false;
            else
            {
                throw new ArgumentException("The 3 points are colinear");
            }
        }

        private static int[] DefineIntersectedEdge(IList<int> nodeIDs, int nodeIdx0, int nodeIdx1)
        {
            if (nodeIDs[nodeIdx0] < nodeIDs[nodeIdx1])
            {
                return new int[] { nodeIDs[nodeIdx0], nodeIDs[nodeIdx1] };
            }
            else
            {
                return new int[] { nodeIDs[nodeIdx1], nodeIDs[nodeIdx0] };
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
    }
}
