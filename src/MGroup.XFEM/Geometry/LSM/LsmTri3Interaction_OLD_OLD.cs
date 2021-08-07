using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.MSolve.Discretization.Mesh;
using System.Diagnostics;

namespace MGroup.XFEM.Geometry.LSM
{
    public class LsmTri3Interaction_OLD_OLD
    {
        public (RelativePositionCurveElement relativePosition, IntersectionMesh2D_OLD intersectionMesh) FindIntersection(
            List<double[]> nodeCoords, List<double> nodeLevelSets)
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

            var intersectionMesh = new IntersectionMesh2D_OLD();
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
                        }
                    }

                    Debug.Assert(intersectionMesh.Vertices.Count == 2);
                    intersectionMesh.Cells.Add((CellType.Line2, new int[] { 0, 1 }));
                    return (RelativePositionCurveElement.Intersecting, intersectionMesh);
                }
            }
            else if (numZeroNodes == 1)
            {
                int nodeZero = nodeLevelSets.FindIndex(phi => phi == 0);
                intersectionMesh.Vertices.Add(nodeCoords[nodeZero]);
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
                    intersectionMesh.Cells.Add((CellType.Line2, new int[] { 0, 1 }));
                    return (RelativePositionCurveElement.Intersecting, intersectionMesh);
                }
            }
            else if (numZeroNodes == 2) // 1 conforming edge
            {
                intersectionMesh.Vertices.Add(nodeCoords[nodeLevelSets.FindIndex(phi => phi == 0)]);
                intersectionMesh.Vertices.Add(nodeCoords[nodeLevelSets.FindLastIndex(phi => phi == 0)]);
                intersectionMesh.Cells.Add((CellType.Line2, new int[] { 0, 1 }));
                return (RelativePositionCurveElement.Conforming, intersectionMesh);
            }
            else // 3 conforming edges
            {
                Debug.Assert(numZeroNodes == 3);
                //TODO: The client should decide whether to log this msg or throw an exception
                Debug.WriteLine(
                    $"Found element that has all its edges conforming to level set curve with ID {int.MinValue}");
                for (int i = 0; i < 3; ++i)
                {
                    intersectionMesh.Vertices.Add(nodeCoords[i]);
                    intersectionMesh.Cells.Add((CellType.Line2, new int[] { i, (i + 1) % 3 }));

                }
                return (RelativePositionCurveElement.Conforming, intersectionMesh);
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
