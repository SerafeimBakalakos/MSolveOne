using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Interpolation;

namespace MGroup.XFEM.Geometry.Mesh
{
    public class DualCartesianMesh2D : DualCartesianMeshBase
    {
        private DualCartesianMesh2D(UniformCartesianMesh2D coarseMesh, UniformCartesianMesh2D fineMesh) 
            : base(2, coarseMesh, fineMesh)
        {
            CoarseToFineElementOffsets = FindElementOffsets(base.multiple);
        }

        public Submesh FindFineNodesEdgesOfCoarseElement(int coarseElementID)
        {
            int[] coarseElementIdx = coarseMesh.GetElementIdx(coarseElementID);
            int[] coarseNodeIDs = coarseMesh.GetElementConnectivity(coarseElementIdx);
            int[] firstCoarseNodeIdx = coarseMesh.GetNodeIdx(coarseNodeIDs[0]);
            int[] firstFineNodeIdx = MapNodeIdxCoarseToFine(firstCoarseNodeIdx);

            //TODO: Improve the performance of the next:
            var fineNodeIDs = new List<int>();
            var edges = new List<(int, int)>();
            for (int j = 0; j <= multiple[1]; ++j)
            {
                for (int i = 0; i <= multiple[0]; ++i)
                {
                    int[] fineNodeIdx = { firstFineNodeIdx[0] + i, firstFineNodeIdx[1] + j };
                    fineNodeIDs.Add(FineMesh.GetNodeID(fineNodeIdx));

                    // Horizontal edges
                    if (i > 0)
                    {
                        int last = fineNodeIDs.Count - 1;
                        edges.Add((fineNodeIDs[last - 1], fineNodeIDs[last]));
                    }
                }
            }

            // Vertical edges
            int verticalEdgesStart = edges.Count;
            int stride = multiple[0] + 1;
            for (int j = 0; j < multiple[1]; ++j)
            {
                for (int i = 0; i <= multiple[0]; ++i)
                {
                    //int down = j * multiple[0] + i;
                    //int up = (j + 1) * multiple[0] + i;
                    int down = j * stride + i;
                    int up = (j + 1) * stride + i;
                    edges.Add((fineNodeIDs[down], fineNodeIDs[up]));
                }
            }

            // Elements //TODO: This is the same for all elements. It only needs to be defined once, perhaps in Submesh
            var elementToEdges = new List<int[]>(FineMesh.NumElementsTotal);
            for (int j = 0; j < multiple[1]; ++j)
            {
                for (int i = 0; i < multiple[0]; ++i)
                {
                    int bottom = j * multiple[0] + i;
                    int top = (j + 1) * multiple[0] + i;
                    int left = verticalEdgesStart + j * (multiple[0] + 1) + i;
                    int right = verticalEdgesStart + j * (multiple[0] + 1) + i + 1;
                    elementToEdges.Add(new int[] { bottom, right, top, left });
                }
            }

            return new Submesh(fineNodeIDs, edges, elementToEdges);
        }

        protected override IIsoparametricInterpolation ElementInterpolation => InterpolationQuad4.UniqueInstance;

        /// <summary>
        /// Let {i, j} (or {i, j, k} in 3D) be the index of a coarse element. Each entry of this list is the offset of the 
        /// index of a fine element, which is included in the coarse element {i, j} (or {i, j, k} in 3D).
        /// </summary>
        protected override List<int[]> CoarseToFineElementOffsets { get; }

        private List<int[]> FindElementOffsets(int[] multiple)
        {
            var elementNeighbors = new List<int[]>();
            for (int j = 0; j < multiple[1]; ++j)
            {
                for (int i = 0; i < multiple[0]; ++i)
                {
                    // Offset from the fine element that has the same first node as the coarse element
                    elementNeighbors.Add(new int[] { i, j });
                }
            }
            return elementNeighbors;
        }

        public class Builder
        {
            private readonly double[] minCoordinates;
            private readonly double[] maxCoordinates;
            private readonly int[] numElementsCoarse;
            private readonly int[] numElementsFine;

            public Builder(double[] minCoordinates, double[] maxCoordinates, int[] numElementsCoarse, int[] numElementsFine)
            {
                this.minCoordinates = minCoordinates;
                this.maxCoordinates = maxCoordinates;
                this.numElementsCoarse = numElementsCoarse;
                this.numElementsFine = numElementsFine;
            }

            public DualCartesianMesh2D BuildMesh()
            {
                var coarseMesh = new UniformCartesianMesh2D.Builder(minCoordinates, maxCoordinates, numElementsCoarse)
                    .SetMajorAxis(0) //TODO: Implement the other options in the mesh class and the builder.
                    .BuildMesh();
                var fineMesh = new UniformCartesianMesh2D.Builder(minCoordinates, maxCoordinates, numElementsFine)
                    .SetMajorAxis(0)
                    .BuildMesh();
                return new DualCartesianMesh2D(coarseMesh, fineMesh);
            }
        }

        /// <summary>
        /// Mesh entities corresponding to a coarse element.
        /// </summary>
        public class Submesh
        {
            public Submesh(List<int> fineNodeIDs, List<(int, int)> fineEdges, List<int[]> fineElementToEdges)
            {
                this.FineNodeIDs = fineNodeIDs;
                this.FineEdgesToNodes = fineEdges;
                this.FineElementToEdges = fineElementToEdges;
            }

            public List<int> FineNodeIDs { get; }

            public List<(int, int)> FineEdgesToNodes { get; }

            public List<int[]> FineElementToEdges { get; }
        }
    }
}
