using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Interpolation;

namespace MGroup.XFEM.Geometry.Mesh
{
    /// <summary>
    /// This work only for a very specific division of 1 Quad into 2 triangles.
    /// </summary>
    public abstract class DualCartesianSimplicialMeshBase : IDualMesh
    {
        protected readonly ICartesianMesh coarseMesh;
        protected readonly IStructuredMesh fineMesh;
        protected readonly int[] multiple;
        protected readonly int numFineElementsPerCoarseElement;
        protected readonly int numSimplicesPerCartesianCell;

        protected DualCartesianSimplicialMeshBase(int dimension, ICartesianMesh coarseMesh, IStructuredMesh fineMesh,
            int numSimplicesPerCartesianCell) 
        {
            this.Dimension = dimension;
            this.coarseMesh = coarseMesh;
            this.fineMesh = fineMesh;
            this.numSimplicesPerCartesianCell = numSimplicesPerCartesianCell;

            multiple = new int[Dimension];
            for (int d = 0; d < Dimension; ++d)
            {
                if (fineMesh.NumNodes[d] < coarseMesh.NumNodes[d])
                {
                    throw new ArgumentException("The number of nodes in each axis of the fine mesh must be greater than"
                        + " or equal to the number of nodes in that axis of the coarse mesh");
                }
                if ((fineMesh.NumNodes[d] - 1) % coarseMesh.NumElements[d] != 0)
                {
                    throw new ArgumentException("The number of elements in each axis of the fine mesh must be a multiple of"
                        + " the number of elements in that axis of the coarse mesh");
                }
                multiple[d] = (fineMesh.NumNodes[d] - 1) / coarseMesh.NumElements[d];
            }

            numFineElementsPerCoarseElement = numSimplicesPerCartesianCell;
            for (int d = 0; d < Dimension; ++d)
            {
                numFineElementsPerCoarseElement *= multiple[d];
            }
        }

        public int Dimension { get; }

        public IStructuredMesh FineMesh => fineMesh;

        public IStructuredMesh CoarseMesh => coarseMesh;

        /// <summary>
        /// Let {i, j} (or {i, j, k} in 3D) be the index of a coarse element. Each entry of this list is the offset of the 
        /// index of a fine cartesian element, which is included in the coarse element {i, j} (or {i, j, k} in 3D).
        /// E.g. 2D-3x3: {0, 0}, {1, 0}, {2, 0}, {0, 1}, {1, 1}, {2, 1}, {0, 2}, {1, 2}, {2, 2}.
        /// E.g. 3D-2x2: {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}.
        /// </summary>
        protected abstract List<int[]> CoarseToFineElementOffsets { get; }

        protected abstract IIsoparametricInterpolation FineElementInterpolation { get; }

        /// <summary>
        /// If the node in the fine mesh does not correspond to a node in the coarse mesh, -1 will be returned
        /// </summary>
        /// <param name="fineNodeID"></param>
        public int MapNodeFineToCoarse(int fineNodeID)
        {
            int[] fineIdx = FineMesh.GetNodeIdx(fineNodeID);
            var coarseIdx = new int[Dimension];
            for (int d = 0; d < Dimension; d++)
            {
                if (fineIdx[d] % multiple[d] != 0) return -1;
                else coarseIdx[d] = fineIdx[d] / multiple[d];
            }
            return CoarseMesh.GetNodeID(coarseIdx);
        }

        public int MapNodeIDCoarseToFine(int coarseNodeID)
        {
            int[] coarseIdx = CoarseMesh.GetNodeIdx(coarseNodeID);
            var fineIdx = MapNodeIdxCoarseToFine(coarseIdx);
            return FineMesh.GetNodeID(fineIdx);
        }

        public int[] MapNodeIdxCoarseToFine(int[] coarseNodeIdx)
        {
            var fineIdx = new int[Dimension];
            for (int d = 0; d < Dimension; d++)
            {
                fineIdx[d] = multiple[d] * coarseNodeIdx[d];
            }
            return fineIdx;
        }

        public int MapElementFineToCoarse(int fineElementID)
        {
            // The last entry is unused, since all sub-simplices belong to the same cartesian cell, defined by the first entries.
            int[] fineIdx = fineMesh.GetElementIdx(fineElementID);  
            var coarseIdx = new int[Dimension];
            for (int d = 0; d < Dimension; d++)
            {
                coarseIdx[d] = fineIdx[d] / multiple[d];
            }
            return coarseMesh.GetElementID(coarseIdx);
        }

        public int[] MapElementCoarseToFine(int coarseElementID)
        {
            int[] coarseIdx = coarseMesh.GetElementIdx(coarseElementID);
            List<int[]> elementOffsets = CoarseToFineElementOffsets;
            var fineElementIDs = new int[numFineElementsPerCoarseElement];
            int i = 0;
            foreach (int[] offset in elementOffsets)
            {
                for (int j = 0; j < numSimplicesPerCartesianCell; ++j)
                {
                    var fineIdx = new int[Dimension + 1];
                    for (int d = 0; d < Dimension; d++)
                    {
                        fineIdx[d] = multiple[d] * coarseIdx[d] + offset[d];
                    }
                    fineIdx[Dimension] = j;
                    int fineID = fineMesh.GetElementID(fineIdx);
                    fineElementIDs[i++] = fineID;
                }
                
            }
            return fineElementIDs;
        }

        //TODO: These mapping and its inverse must also work for points on edges of the fine and coarse mesh.
        public abstract double[] MapPointFineNaturalToCoarseNatural(int[] fineElementIdx, double[] coordsFineNatural);

        public abstract DualMeshPoint CalcShapeFunctions(int coarseElementID, double[] coarseNaturalCoords);

        
    }
}
