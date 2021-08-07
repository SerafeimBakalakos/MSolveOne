using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Interpolation;

//TODO: add additional functionality found in the individual meshes: selecting major axis, order of element nodes, etc.
namespace MGroup.XFEM.Geometry.Mesh
{
    public abstract class DualCartesianMeshBase : IDualMesh
    {
        protected readonly ICartesianMesh coarseMesh;
        protected readonly ICartesianMesh fineMesh;
        protected readonly int[] multiple;

        protected DualCartesianMeshBase(int dimension, ICartesianMesh coarseMesh, ICartesianMesh fineMesh)
        {
            this.Dimension = dimension;
            this.coarseMesh = coarseMesh;
            this.fineMesh = fineMesh;

            multiple = new int[Dimension];
            for (int d = 0; d < Dimension; ++d)
            {
                if (fineMesh.NumElements[d] < coarseMesh.NumElements[d])
                {
                    throw new ArgumentException("The number of elements in each axis of the fine mesh must be greater than"
                        + " or equal to the number of elements in that axis of the coarse mesh");
                }
                if (fineMesh.NumElements[d] % coarseMesh.NumElements[d] != 0)
                {
                    throw new ArgumentException("The number of elements in each axis of the fine mesh must be a multiple of"
                        + " the number of elements in that axis of the coarse mesh");
                }
                multiple[d] = fineMesh.NumElements[d] / coarseMesh.NumElements[d];
            }
        }

        public int Dimension { get; }

        public IStructuredMesh FineMesh => fineMesh;

        public IStructuredMesh CoarseMesh => coarseMesh;

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
            var fineElementIDs = new int[elementOffsets.Count];
            for (int i = 0; i < elementOffsets.Count; ++i)
            {
                int[] offset = elementOffsets[i];
                var fineIdx = new int[Dimension];
                for (int d = 0; d < Dimension; d++)
                {
                    fineIdx[d] = multiple[d] * coarseIdx[d] + offset[d];
                }
                int fineID = fineMesh.GetElementID(fineIdx);
                fineElementIDs[i] = fineID;
            }
            return fineElementIDs;
        }

        //TODO: These mapping and its inverse must also work for points on edges of the fine and coarse mesh.
        public double[] MapPointFineNaturalToCoarseNatural(int[] fineElementIdx, double[] coordsFineNatural)
        {
            var coordsCoarseNatural = new double[Dimension];
            for (int d = 0; d < Dimension; ++d)
            {
                // Let: 
                // x = coarse natural coordinate
                // x0 = coarse natural coordinate starting from the min point of the axis (-1): x0 = 1 + x
                // dx = max - min of coarse natural coordinate axis: dx = +1 - (-1) = 2
                // m = multiplicity of fine elements per coarse element in this axis
                // i = index of fine element starting from the one at the min of the axis: i = 0, 1, ... m-1
                // r = fine natural coordinate
                // r0 = fine natural coordinate starting from the min point of the axis (-1): r0 = 1 + r
                // dr = max - min of fine natural coordinate axis: dr = dx/m
                // x0 = i * dr + r0/m => x = i * dx / m + (1+r) / m - 1

                int i = fineElementIdx[d] % multiple[d];
                coordsCoarseNatural[d] = (i * 2.0 + coordsFineNatural[d] + 1.0) / multiple[d] - 1.0;
            }
            return coordsCoarseNatural;
        }

        //TODO: Perhaps split this into a function that maps coarse -> fine and one that calculates shape functions. 
        //      The shape functions should probably be done someplace else. No need to couple mesh classes with interpolations.
        //      Do not expose this subElementsIdx. Instead use element IDs (which may need multiple conversions ID -> idx)
        public DualMeshPoint CalcShapeFunctions(int coarseElementID, double[] coarseNaturalCoords)
        {
            // Find the fine element containing that point and the natural coordinates in that element
            var subElementsIdx = new int[Dimension];
            var fineNaturalCoords = new double[Dimension];
            for (int d = 0; d < Dimension; ++d)
            {
                // Let: 
                // x = coarse natural coordinate
                // x0 = coarse natural coordinate starting from the min point of the axis (-1): x0 = 1 + x
                // dx = max - min of coarse natural coordinate axis: dx = +1 - (-1) = 2
                // m = multiplicity of fine elements per coarse element in this axis
                // i = index of fine element starting from the one at the min of the axis: i = 0, 1, ... m-1
                // r = fine natural coordinate
                // r0 = fine natural coordinate starting from the min point of the axis (-1): r0 = 1 + r
                // dr = max - min of fine natural coordinate axis: dr = dx/m
                // x0 = i * dr + r0/m => r = m * x - 2 * i + m - 1
                double dx = 2.0; // [-1, 1]
                double x0 = 1 + coarseNaturalCoords[d];
                double m = multiple[d];
                subElementsIdx[d] = (int)Math.Floor(x0 * m / dx);
                fineNaturalCoords[d] = m * coarseNaturalCoords[d] - 2 * subElementsIdx[d] + m - 1;
            }

            // Calculate the shape functions in this fine element
            double[] shapeFunctions = ElementInterpolation.EvaluateFunctionsAt(fineNaturalCoords);

            var result = new DualMeshPoint();
            result.FineNaturalCoordinates = fineNaturalCoords;
            result.FineShapeFunctions = shapeFunctions;

            result.FineElementIdx = new int[Dimension];
            int[] coarseElementIdx = coarseMesh.GetElementIdx(coarseElementID);
            for (int d = 0; d < Dimension; ++d)
            {
                result.FineElementIdx[d] = coarseElementIdx[d] * multiple[d] + subElementsIdx[d];
            }

            return result;
        }

        protected abstract IIsoparametricInterpolation ElementInterpolation { get; }

        /// <summary>
        /// Let {i, j} (or {i, j, k} in 3D) be the index of a coarse element. Each entry of this list is the offset of the 
        /// index of a fine element, which is included in the coarse element {i, j} (or {i, j, k} in 3D).
        /// E.g. 2D-3x3: {0, 0}, {1, 0}, {2, 0}, {0, 1}, {1, 1}, {2, 1}, {0, 2}, {1, 2}, {2, 2}.
        /// E.g. 3D-2x2: {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}.
        /// </summary>
        protected abstract List<int[]> CoarseToFineElementOffsets { get; } 
    }
}
