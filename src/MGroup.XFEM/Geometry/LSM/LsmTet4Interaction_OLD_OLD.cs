using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.MSolve.Discretization.Mesh;
using System.Diagnostics;

//TODO: Make sure all triangles have the same orientation. This orientation must be the same with triangles from other elements!
//      This could be done by pointing always towards a positive node. Also apply this to 2D.
//TODO: Make these intersections as smooth as the contours in ParaView
//TODO: Optimizations are possible, but may mess up readability. E.g. depending on the case, we can target specific edges that 
//      are intersected, instead of checking all of them
//TODO: For the common case, where the level set intersects the Tet4 into a triangle, there is the corner case that this triangle 
//      is extremely close to the node. In that case, it is probably safer to regard this as "Tangent". What happens if only 1 
//      or only 2 of the triangle vertices coincide with the node? 
namespace MGroup.XFEM.Geometry.LSM
{
    public class LsmTet4Interaction_OLD_OLD
    {
        public (RelativePositionCurveElement relativePosition, IntersectionMesh3D_OLD intersectionMesh) FindIntersection(
            List<double[]> nodeCoords, List<double> nodeLevelSets)
        {
            Debug.Assert(nodeCoords.Count == 4);
            Debug.Assert(nodeLevelSets.Count == 4);

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
                    var intersectionMesh = new IntersectionMesh3D_OLD();
                    return (RelativePositionCurveElement.Disjoint, intersectionMesh);
                }
                else if((numPosNodes == 1) || (numNegNodes == 1))
                {
                    // Intersection. 3 intersection points on edges of the single positive/negative node.
                    // The intersection mesh consists of a single triangle.
                    var intersectionMesh = new IntersectionMesh3D_OLD();
                    List<double[]> intersections = FindEdgeIntersections(nodeCoords, nodeLevelSets);
                    Debug.Assert(intersections.Count == 3);
                    foreach (double[] point in intersections)
                    {
                        intersectionMesh.Vertices.Add(point);
                    }
                    intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
                    return (RelativePositionCurveElement.Intersecting, intersectionMesh);
                }
                else
                {
                    // Intersection. 4 intersection points on edges of the 2 positive nodes that connect them with the 2 
                    // negative nodes. The intersection mesh consists of 2 triangles.
                    Debug.Assert((numPosNodes == 2) && (numNegNodes == 2));
                    IntersectionMesh3D_OLD intersectionMesh = Process2Pos2NegCase(nodeCoords, nodeLevelSets);
                    return (RelativePositionCurveElement.Intersecting, intersectionMesh);
                    
                }
            }
            else if (numZeroNodes == 1)
            {
                var intersectionMesh = new IntersectionMesh3D_OLD();
                int nodeZero = nodeLevelSets.FindIndex(phi => phi == 0);
                intersectionMesh.Vertices.Add(nodeCoords[nodeZero]);

                if ((numPosNodes == 0) || (numNegNodes == 0))
                {
                    // Tangent. The zero node is the only common point.
                    return (RelativePositionCurveElement.Tangent, intersectionMesh);
                }
                else
                {
                    // Intersection. A single positive or negative node. 2 intersection points on its edges and the zero node.
                    // The intersection mesh consists of a single triangle.
                    Debug.Assert(((numPosNodes == 1) && (numNegNodes == 2)) || ((numPosNodes == 2) && (numNegNodes == 1)));
                    List<double[]> intersections = FindEdgeIntersections(nodeCoords, nodeLevelSets);
                    Debug.Assert(intersections.Count == 2);
                    intersectionMesh.Vertices.Add(intersections[0]);
                    intersectionMesh.Vertices.Add(intersections[1]);
                    intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
                    return (RelativePositionCurveElement.Intersecting, intersectionMesh);
                }
            }
            else if (numZeroNodes == 2)
            {
                var intersectionMesh = new IntersectionMesh3D_OLD();
                int nodeZero0 = nodeLevelSets.FindIndex(phi => phi == 0);
                int nodeZero1 = nodeLevelSets.FindLastIndex(phi => phi == 0);
                intersectionMesh.Vertices.Add(nodeCoords[nodeZero0]);
                intersectionMesh.Vertices.Add(nodeCoords[nodeZero1]);

                if ((numPosNodes == 0) || (numNegNodes == 0))
                {
                    // Tangent. The 2 zero nodes define a single common line segment, but no cell.
                    //intersectionMesh.Cells.Add((CellType.Line, new int[] { 0, 1 })); // Nope, edges are different than cells.
                    return (RelativePositionCurveElement.Tangent, intersectionMesh);
                }
                else
                {
                    // Intersection. 2 zero nodes and 1 intersection point on the edge connecting the positive and negative edge.
                    // The intersection mesh consists of a single triangle.
                    Debug.Assert((numPosNodes == 1) && (numPosNodes == 1));
                    List<double[]> intersections = FindEdgeIntersections(nodeCoords, nodeLevelSets);
                    Debug.Assert(intersections.Count == 1);
                    intersectionMesh.Vertices.Add(intersections[0]);
                    intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
                    return (RelativePositionCurveElement.Intersecting, intersectionMesh);
                }
            }
            else if (numZeroNodes == 3)
            {
                // Conforming. The intersection mesh consists of the face connecting the 3 zero nodes.
                var intersectionMesh = new IntersectionMesh3D_OLD();
                for (int i = 0; i < 4; ++i)
                {
                    if (nodeLevelSets[i] == 0)
                    {
                        intersectionMesh.Vertices.Add(nodeCoords[i]);
                    }
                }
                intersectionMesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
                return (RelativePositionCurveElement.Conforming, intersectionMesh);
            }
            else
            {
                // Conforming. All faces are conforming, which means that the lsm surface hugs the Tet4 and nothing else.
                Debug.Assert(numZeroNodes == 4);
                //TODO: The client should decide whether to log this msg or throw an exception
                Debug.WriteLine(
                    $"Found element that has all its faces conforming to level set surface with ID {int.MinValue}");
                var intersectionMesh = new IntersectionMesh3D_OLD();
                for (int i = 0; i < 4; ++i)
                {
                    intersectionMesh.Vertices.Add(nodeCoords[i]);
                    intersectionMesh.Cells.Add((CellType.Tri3, new int[] { i, (i + 1) % 3, (i + 2) % 3 }));
                }
                return (RelativePositionCurveElement.Conforming, intersectionMesh);
            }
        }

        private IntersectionMesh3D_OLD Process2Pos2NegCase(List<double[]> nodeCoords, List<double> nodeLevelSets)
        {
            // Intersection. 4 intersection points on edges of the 2 positive nodes that connect them with the 2 
            // negative nodes. The intersection mesh consists of 2 triangles.

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
            var intersections = new List<double[]>();
            intersections.Add(Interpolate(
                nodeLevelSets[negNodes[0]], nodeCoords[negNodes[0]], nodeLevelSets[posNodes[0]], nodeCoords[posNodes[0]]));
            intersections.Add(Interpolate(
                nodeLevelSets[negNodes[0]], nodeCoords[negNodes[0]], nodeLevelSets[posNodes[1]], nodeCoords[posNodes[1]]));
            intersections.Add(Interpolate(
                nodeLevelSets[negNodes[1]], nodeCoords[negNodes[1]], nodeLevelSets[posNodes[1]], nodeCoords[posNodes[1]]));
            intersections.Add(Interpolate(
                nodeLevelSets[negNodes[1]], nodeCoords[negNodes[1]], nodeLevelSets[posNodes[0]], nodeCoords[posNodes[0]]));

            //List<double[]> intersections = FindEdgeIntersections(nodeCoords, nodeLevelSets);
            //Debug.Assert(intersections.Count == 4);
            var intersectionMesh = new IntersectionMesh3D_OLD();
            foreach (double[] point in intersections)
            {
                intersectionMesh.Vertices.Add(point);
            }

            List<int[]> triangles = Delauny4Points3D(intersections);
            Debug.Assert(triangles.Count == 2);
            intersectionMesh.Cells.Add((CellType.Tri3, triangles[0]));
            intersectionMesh.Cells.Add((CellType.Tri3, triangles[1]));
            return intersectionMesh;
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

        private static List<double[]> FindEdgeIntersections(List<double[]> nodeCoords, List<double> nodalLevelSets)
        {
            var intersections = new List<double[]>();
            for (int i = 0; i < 4; ++i)
            {
                for (int j = i + 1; j < 4; ++j)
                {
                    if (nodalLevelSets[i] * nodalLevelSets[j] < 0)
                    {
                        int nodeNeg, nodePos;
                        if (nodalLevelSets[i] < nodalLevelSets[j])
                        {
                            nodeNeg = i;
                            nodePos = j;
                        }
                        else
                        {
                            nodeNeg = j;
                            nodePos = i;
                        }
                        double[] intersection = Interpolate(nodalLevelSets[nodeNeg], nodeCoords[nodeNeg],
                            nodalLevelSets[nodePos], nodeCoords[nodePos]);
                        intersections.Add(intersection);
                    }
                }
            }
            return intersections;
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
