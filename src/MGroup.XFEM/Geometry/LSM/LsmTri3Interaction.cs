using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using System.Diagnostics;

//TODO: Remove duplicate code between this and the Tet4 version.
//TODO: Clean up recursion logic and use helper methods (with delegates). If possible, add these helepers to the base class.
namespace MGroup.XFEM.Geometry.LSM
{
    public class LsmTri3Interaction  : LsmCellInteractionBase
    {
        public LsmTri3Interaction(IList<int> nodeIDs, List<double[]> nodeCoords, List<double> nodeLevelSets, double tolerance)
            : base(2, 3, nodeIDs, nodeCoords, nodeLevelSets, tolerance)
        {
            Edges = new List<(int nodeIdx0, int nodeIdx1)>();
            Edges.Add((0, 1));
            Edges.Add((1, 2));
            Edges.Add((2, 0));
        }

        protected override List<(int nodeIdx0, int nodeIdx1)> Edges { get; }

        public override void Resolve()
        {
            (int numZeroNodes, int numPosNodes, int numNegNodes) = CountNodes();

            if (numZeroNodes == 0)
            {
                if ((numPosNodes == 0) || (numNegNodes == 0)) // Disjoint
                {
                    ProcessCase0Zeros3SameSigns();
                }
                else // 2 intersection points
                {
                    ProcessIntersectionCase(ProcessCase0Zeros2SameSigns1Different, false);
                    return;
                    //if (!areLevelSetsAdjusted)
                    //{
                    //    AdjustLevelSetsToAvoidDegenerateIntersections();

                    //    if (areLevelSetsAdjusted)
                    //    {
                    //        // Level sets needed adjusting. Find and process the new interaction case, based on the new level sets.
                    //        //TODO: Now only some cases are possible, so I could optimize that determination.
                    //        Resolve(); // recurse but only 1 level
                    //        return; // do not do anything else after finishing the recursive level.
                    //    }
                    //    else
                    //    {
                    //        // Level sets were ok after all. Proceed to find intersections normally.
                    //        ProcessCase0Zeros2SameSigns1Different();
                    //    }
                    //}
                    //else
                    //{
                    //    throw new Exception("This should not have happened. Reaching this means that level sets were modified," +
                    //        " thus there is at least 1 zero node.");
                    //}
                }
            }
            else if (numZeroNodes == 1)
            {
                if ((numPosNodes == 0) || (numNegNodes == 0)) // Tangent (only 1 common point)
                {
                    ProcessCase1Zero2SameSigns();
                }
                else // 1 intersection point and 1 node
                {
                    ProcessIntersectionCase(ProcessCase1Zero1Pos1Neg, true);
                    return;

                    //if (!areLevelSetsAdjusted)
                    //{
                    //    AdjustLevelSetsToAvoidDegenerateIntersections();

                    //    if (areLevelSetsAdjusted)
                    //    {
                    //        // Level sets needed adjusting. Find and process the new interaction case, based on the new level sets.
                    //        //TODO: Now only some cases are possible, so I could optimize that determination.
                    //        Resolve(); // recurse but only 1 level
                    //        return; // do not do anything else after finishing the recursive level.
                    //    }
                    //    else
                    //    {
                    //        // Level sets were ok after all. Proceed to find intersections normally.
                    //        ProcessCase1Zero1Pos1Neg();
                    //    }
                    //}
                    //else
                    //{
                    //    // It is possible to reach this point by adjusting the level sets in another case.
                    //    ProcessCase1Zero1Pos1Neg();
                    //}
                }
            }
            else if (numZeroNodes == 2) // 1 conforming edge
            {
                ProcessCase2Zeros1Nonzero();
            }
            else // 3 conforming edges
            {
                Debug.Assert(numZeroNodes == 3);
                ProcessCase3Zeros();
            }
        }

        

        /// <summary>
        /// The returned normal is not necessarily unit, but uniquely defines the orientation of the cell.
        /// </summary>
        protected override double[] CalcNormalOfCell(int[] cellConnectivity)
        {
            double[] pA = Mesh.Vertices[cellConnectivity[0]];
            double[] pB = Mesh.Vertices[cellConnectivity[1]];
            double[] normal = { -(pB[1] - pA[1]), pB[0] - pA[0] }; // normal to AB, pi/2 counter-clockwise
            //normal.ScaleIntoThis(1.0 / normal.Norm2()); // Not needed here
            return normal;
        }

        private void ProcessCase0Zeros3SameSigns()
        {
            Position = RelativePositionCurveElement.Disjoint;
        }

        private void ProcessCase0Zeros2SameSigns1Different()
        {
            Position = RelativePositionCurveElement.Intersecting;

            AddEdgeIntersectionsToMesh();
            Debug.Assert(Mesh.Vertices.Count == 2);

            Mesh.Cells.Add((CellType.Line2, new int[] { 0, 1 }));
            FixCellsOrientation();
        }

        private void ProcessCase1Zero2SameSigns()
        {
            Position = RelativePositionCurveElement.Tangent;
        }

        private void ProcessCase1Zero1Pos1Neg()
        {
            if (!areLevelSetsAdjusted)
            {
                AdjustLevelSetsToAvoidDegenerateIntersections();

                if (areLevelSetsAdjusted)
                {
                    // Level sets needed adjusting. Find and process the new interaction case, based on the new level sets.
                    //TODO: Now only some cases are possible, so I could optimize that determination.
                    Resolve();
                }
            }

            Position = RelativePositionCurveElement.Intersecting;

            int nodeZero = nodeLevelSets.FindIndex(phi => phi == 0);
            AddConformingNodeToMesh(nodeZero);

            int nodeNeg = nodeLevelSets.FindIndex(phi => phi < 0);
            int nodePos = nodeLevelSets.FindIndex(phi => phi > 0);
            double[] intersection = Interpolate(nodeLevelSets[nodeNeg], nodeCoords[nodeNeg],
                nodeLevelSets[nodePos], nodeCoords[nodePos]);
            Mesh.Vertices.Add(intersection);
            Mesh.IntersectedEdges.Add(DefineIntersectedEdge(nodeNeg, nodePos));

            Mesh.Cells.Add((CellType.Line2, new int[] { 0, 1 }));
            FixCellsOrientation();
        }

        private void ProcessCase2Zeros1Nonzero()
        {
            Position = RelativePositionCurveElement.Conforming;
            int nodeZero0 = nodeLevelSets.FindIndex(phi => phi == 0);
            int nodeZero1 = nodeLevelSets.FindLastIndex(phi => phi == 0);
            AddConformingNodeToMesh(nodeZero0);
            AddConformingNodeToMesh(nodeZero1);
            Mesh.Cells.Add((CellType.Line2, new int[] { 0, 1 }));
            FixCellsOrientation();
        }

        private void ProcessCase3Zeros()
        {
            //TODO: The client should decide whether to log this msg or throw an exception
            Debug.WriteLine(
                $"Found element that has all its edges conforming to level set curve with ID {int.MinValue}." +
                $" This usually indicates an error. It may also cause problems if the triangle nodes are not given in" +
                $" counter-clockwise order.");
            Position = RelativePositionCurveElement.Conforming;
            for (int i = 0; i < 3; ++i)
            {
                AddConformingNodeToMesh(i);

                // We assume that i) the level set encircles this element and intersects no other, ii) the interior is 
                // negative and the exterior positive, iii) the triangle's nodes are in counter-clockwise order. 
                // Thus if we traverse each edge is i+1 -> i, then the normal will point outside, meaning towards positive.
                int j = (i + 1) % 3;
                Mesh.Cells.Add((CellType.Line2, new int[] { j, i }));
            }
        }

        public class Factory : ILsmElementInteractionFactory
        {
            public ILsmCellInteraction CreateNewInteraction(
                IList<int> nodeIDs, List<double[]> nodeCoords, List<double> nodeLevelSets, double tolerance)
            {
                var interaction = new LsmTri3Interaction(nodeIDs, nodeCoords, nodeLevelSets, tolerance);
                interaction.Resolve();
                return interaction;
            }
        }
    }
}
