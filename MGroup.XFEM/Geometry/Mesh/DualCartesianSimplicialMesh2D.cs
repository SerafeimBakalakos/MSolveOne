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
    public class DualCartesianSimplicialMesh2D : DualCartesianSimplicialMeshBase
    {
        private DualCartesianSimplicialMesh2D(UniformCartesianMesh2D coarseMesh, UniformSimplicialMesh2D fineMesh)
            : base(2, coarseMesh, fineMesh, 2)
        {
            CoarseToFineElementOffsets = FindElementOffsets(multiple);
        }

        protected override IIsoparametricInterpolation FineElementInterpolation { get; } = InterpolationTri3.UniqueInstance;

        /// <summary>
        /// Let {i, j} (or {i, j, k} in 3D) be the index of a coarse element. Each entry of this list is the offset of the 
        /// index of a fine cartesian element, which is included in the coarse element {i, j} (or {i, j, k} in 3D).
        /// E.g. 2D-3x3: {0, 0}, {1, 0}, {2, 0}, {0, 1}, {1, 1}, {2, 1}, {0, 2}, {1, 2}, {2, 2}.
        /// E.g. 3D-2x2: {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}.
        /// </summary>
        protected override List<int[]> CoarseToFineElementOffsets { get; }

        //TODO: These mapping and its inverse must also work for points on edges/faces of the fine and coarse mesh.
        public override double[] MapPointFineNaturalToCoarseNatural(int[] fineElementIdx, double[] coordsFineNatural)
        {
            // Map from the fine triangle to the fine quad. 
            // To prove these formulas, use the interpolation from natural triangle to natural quad system.
            var coordsFineQuad = new double[2];
            if (fineElementIdx[2] == 0)
            {
                // |\
                // | \
                // |__\
                coordsFineQuad[0] = 2 * coordsFineNatural[0] - 1;
                coordsFineQuad[1] = 2 * coordsFineNatural[1] - 1;
            }
            else
            {
                Debug.Assert(fineElementIdx[2] == 1);

                // ___
                // \  |
                //  \ |
                //   \|
                coordsFineQuad[0] = 1 - 2 * coordsFineNatural[0];
                coordsFineQuad[1] = 1 - 2 * coordsFineNatural[1];
            }

            // Map from the fine quad to the coarse quad
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
                coordsCoarseNatural[d] = (i * 2.0 + coordsFineQuad[d] + 1.0) / multiple[d] - 1.0;
            }
            return coordsCoarseNatural;
        }

        public override DualMeshPoint CalcShapeFunctions(int coarseElementID, double[] coarseNaturalCoords)
        {
            // Find the quad of the fine mesh containing that point and the natural coordinates in that quad
            var subElementsIdx = new int[Dimension];
            var coordsFineQuad = new double[Dimension];
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
                coordsFineQuad[d] = m * coarseNaturalCoords[d] - 2 * subElementsIdx[d] + m - 1;
            }

            // Map from the fine quad to the fine triangle.
            var coordsFineTriangle = new double[2];
            int subtriangle = (coordsFineQuad[0] + coordsFineQuad[1] <= 0) ? 0 : 1;
            if (subtriangle == 0) // under (or on) the diagonal of the quad
            {
                // |\
                // | \
                // |__\
                coordsFineTriangle[0] = 0.5 * (coordsFineQuad[0] + 1);
                coordsFineTriangle[1] = 0.5 * (coordsFineQuad[1] + 1);
            }
            else // over the diagonal of the quad
            {
                // ___
                // \  |
                //  \ |
                //   \|
                coordsFineTriangle[0] = 0.5 * (1 - coordsFineQuad[0]);
                coordsFineTriangle[1] = 0.5 * (1 - coordsFineQuad[1]);
            }

            // Calculate the shape functions in this fine element
            double[] shapeFunctions = FineElementInterpolation.EvaluateFunctionsAt(coordsFineTriangle);

            var result = new DualMeshPoint();
            result.FineNaturalCoordinates = coordsFineTriangle;
            result.FineShapeFunctions = shapeFunctions;

            result.FineElementIdx = new int[Dimension + 1];
            int[] coarseElementIdx = coarseMesh.GetElementIdx(coarseElementID);
            for (int d = 0; d < Dimension; ++d)
            {
                result.FineElementIdx[d] = coarseElementIdx[d] * multiple[d] + subElementsIdx[d];
            }
            result.FineElementIdx[Dimension] = subtriangle;

            return result;
        }

        private List<int[]> FindElementOffsets(int[] multiple)
        {
            var offsets = new List<int[]>();
            for (int j = 0; j < multiple[1]; ++j)
            {
                for (int i = 0; i < multiple[0]; ++i)
                {
                    // Offset from the fine element that has the same first node as the coarse element
                    offsets.Add(new int[] { i, j });
                }
            }
            return offsets;
        }

        public class Builder
        {
            private readonly double[] minCoordinates;
            private readonly double[] maxCoordinates;
            private readonly int[] numNodesCoarse;
            private readonly int[] numNodesFine;

            public Builder(double[] minCoordinates, double[] maxCoordinates, int[] numNodesCoarse, int[] numNodesFine)
            {
                this.minCoordinates = minCoordinates;
                this.maxCoordinates = maxCoordinates;
                this.numNodesCoarse = numNodesCoarse;
                this.numNodesFine = numNodesFine;
            }

            public DualCartesianSimplicialMesh2D BuildMesh()
            {
                int[] numElementsCoarse = { numNodesCoarse[0] - 1, numNodesCoarse[1] - 1 };
                var coarseMesh = new UniformCartesianMesh2D.Builder(minCoordinates, maxCoordinates, numElementsCoarse)
                    .SetMajorAxis(0) //TODO: Implement the other options in the mesh class and the builder.
                    .BuildMesh();
                var fineMesh = new UniformSimplicialMesh2D.Builder(minCoordinates, maxCoordinates, numNodesFine)
                    .SetMajorAxis(0)
                    .BuildMesh();
                return new DualCartesianSimplicialMesh2D(coarseMesh, fineMesh);
            }
        }
    }
}
