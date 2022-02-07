using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Interpolation;

//TODO: Remove redundancies between this, the 2D version and the purely cartesian versions.
namespace MGroup.XFEM.Geometry.Mesh
{
    /// <summary>
    /// This work only for a very specific division of 1 Hexa8 into 6 Tet4.
    /// </summary>
    public class DualCartesianSimplicialMesh3D : DualCartesianSimplicialMeshBase
    {
        public DualCartesianSimplicialMesh3D(UniformCartesianMesh3D coarseMesh, UniformSimplicialMesh3D fineMesh)
            : base(3, coarseMesh, fineMesh, 6)
        {
            CoarseToFineElementOffsets = FindElementOffsets(multiple);
        }

        protected override IIsoparametricInterpolation FineElementInterpolation { get; } = InterpolationTet4.UniqueInstance;

        /// <summary>
        /// Let {i, j} (or {i, j, k} in 3D) be the index of a coarse element. Each entry of this list is the offset of the 
        /// index of a fine cartesian element, which is included in the coarse element {i, j} (or {i, j, k} in 3D).
        /// E.g. 2D-3x3: {0, 0}, {1, 0}, {2, 0}, {0, 1}, {1, 1}, {2, 1}, {0, 2}, {1, 2}, {2, 2}.
        /// E.g. 3D-2x2: {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}.
        /// </summary>
        protected override List<int[]> CoarseToFineElementOffsets { get; }

        //TODO: These mapping and its inverse must also work for points on edges of the fine and coarse mesh.
        public override double[] MapPointFineNaturalToCoarseNatural(int[] fineElementIdx, double[] coordsFineNatural)
        {
            // Map from the fine triangle to the fine quad. 
            int subtetraIdx = fineElementIdx[Dimension];
            double[] coordsFineHexa = MapCoordsTet4ToFineHexa8(subtetraIdx, coordsFineNatural);

            // Map from the fine hexa to the coarse hexa
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
                coordsCoarseNatural[d] = (i * 2.0 + coordsFineHexa[d] + 1.0) / multiple[d] - 1.0;
            }
            return coordsCoarseNatural;
        }

        public override DualMeshPoint CalcShapeFunctions(int coarseElementID, double[] coarseNaturalCoords)
        {
            // Find the hexa of the fine mesh containing that point and the natural coordinates in that hexa
            var subElementsIdx = new int[Dimension];
            var coordsFineHexa = new double[Dimension];
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
                coordsFineHexa[d] = m * coarseNaturalCoords[d] - 2 * subElementsIdx[d] + m - 1;
            }

            // Map from the fine hexa to the fine tetra.
            int subtetraIdx = FindSubtetraIndex(coordsFineHexa);
            double[] coordsFineTet4 = MapCoordsFineHexa8ToTet4(subtetraIdx, coordsFineHexa);
            
            // Calculate the shape functions in this fine element
            double[] shapeFunctions = FineElementInterpolation.EvaluateFunctionsAt(coordsFineTet4);

            var result = new DualMeshPoint();
            result.FineNaturalCoordinates = coordsFineTet4;
            result.FineShapeFunctions = shapeFunctions;

            result.FineElementIdx = new int[Dimension + 1];
            int[] coarseElementIdx = coarseMesh.GetElementIdx(coarseElementID);
            for (int d = 0; d < Dimension; ++d)
            {
                result.FineElementIdx[d] = coarseElementIdx[d] * multiple[d] + subElementsIdx[d];
            }
            result.FineElementIdx[Dimension] = subtetraIdx;

            return result;
        }

        private double[] MapCoordsTet4ToFineHexa8(int subtetraIndex, double[] coordsTet4)
        {
            // To prove these formulas, use the interpolation from natural Tet4 to natural Hexa8.
            double r = coordsTet4[0];
            double s = coordsTet4[1];
            double t = coordsTet4[2];
            double xi, eta, zeta;
            if (subtetraIndex == 0)
            {
                xi = 2 * r + 2 * s - 1;
                eta = 2 * s - 1;
                zeta = 2 * t - 1;
            }
            else if (subtetraIndex == 1)
            {
                xi = 2 * r - 1;
                eta = 2 * r + 2 * s - 1;
                zeta = 2 * t - 1;
            }
            else if (subtetraIndex == 2)
            {
                xi = 1 - 2 * s;
                eta = 2 * r - 1;
                zeta = 2 * s + 2 * t - 1;
            }
            else if (subtetraIndex == 3)
            {
                xi = 1 - 2 * s;
                eta = 1 - 2 * r - 2 * s;
                zeta = 1 - 2 * t;
            }
            else if (subtetraIndex == 4)
            {
                xi = 1 - 2 * r - 2 * s;
                eta = 1 - 2 * r;
                zeta = 1 - 2 * t;
            }
            else
            {
                Debug.Assert(subtetraIndex == 5);
                xi = 2 * s - 1;
                eta = 1 - 2 * r;
                zeta = 2 * r + 2 * t - 1;
            }
            return new double[] { xi, eta, zeta };
        }

        private double[] MapCoordsFineHexa8ToTet4(int subtetraIndex, double[] coordsHexa8)
        {
            double xi = coordsHexa8[0];
            double eta = coordsHexa8[1];
            double zeta = coordsHexa8[2];
            double r, s, t;
            if (subtetraIndex == 0)
            {
                r = (xi - eta) / 2;
                s = (eta + 1) / 2;
                t = (zeta + 1) / 2;
            }
            else if (subtetraIndex == 1)
            {
                r = (xi + 1) / 2;
                s = (eta - xi) / 2;
                t = (zeta + 1) / 2;
            }
            else if (subtetraIndex == 2)
            {
                r = (eta + 1) / 2;
                s = (1 - xi) / 2;
                t = (xi + zeta) / 2;
            }
            else if (subtetraIndex == 3)
            {
                r = (xi - eta) / 2;
                s = (1 - xi) / 2;
                t = (1 - zeta) / 2;
            }
            else if (subtetraIndex == 4)
            {
                r = (1 - eta) / 2;
                s = (eta - xi) / 2;
                t = (1 - zeta) / 2;
            }
            else
            {
                Debug.Assert(subtetraIndex == 5);
                r = (1 - eta) / 2;
                s = (xi + 1) / 2;
                t = (eta + zeta) / 2;
            }
            return new double[] { r, s, t };
        }

        private int FindSubtetraIndex(double[] coordsFineHexa8)
        {
            // Use the 3 planes that divide the Hexa8 into 6 Tet4. Their equations are:
            double a = coordsFineHexa8[1] - coordsFineHexa8[0];
            double b = coordsFineHexa8[0] + coordsFineHexa8[2];
            double c = coordsFineHexa8[1] + coordsFineHexa8[2];

            // WARNING: The equality signs are carefully and intenionally placed there. Do not fuck with them or anything else. 
            // Actually do not try to fix or improve this, without writing tests for each conceivable case.
            if (a <= 0)
            {
                if (b < 0)
                {
                    if (c <= 0)
                    {
                        return 0;
                    }
                    else
                    {
                        throw new Exception("This should not have happened");
                    }
                }
                else
                {
                    if (c <= 0)
                    {
                        return 2;
                    }
                    else
                    {
                        return 3;
                    }
                }
            }
            else
            {
                if (b <= 0)
                {
                    if (c <= 0)
                    {
                        return 1;
                    }
                    else
                    {
                        return 5;
                    }
                }
                else
                {
                    if (c < 0)
                    {
                        throw new Exception("This should not have happened");
                    }
                    else
                    {
                        return 4;
                    }
                }
            }
        }

        private List<int[]> FindElementOffsets(int[] multiple)
        {
            var offsets = new List<int[]>();
            for (int k = 0; k < multiple[2]; ++k)
            {
                for (int j = 0; j < multiple[1]; ++j)
                {
                    for (int i = 0; i < multiple[0]; ++i)
                    {
                        // Offset from the fine element that has the same first node as the coarse element
                        offsets.Add(new int[] { i, j, k });
                    }
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

            public DualCartesianSimplicialMesh3D BuildMesh()
            {
                int[] numElementsCoarse = { numNodesCoarse[0] - 1, numNodesCoarse[1] - 1, numNodesCoarse[2] - 1 };
                var coarseMesh = new UniformCartesianMesh3D.Builder(minCoordinates, maxCoordinates, numElementsCoarse)
                    .SetMajorMinorAxis(0, 2) //TODO: Implement the other options in the mesh class and the builder.
                    .BuildMesh();
                var fineMesh = new UniformSimplicialMesh3D.Builder(minCoordinates, maxCoordinates, numNodesFine)
                    .SetMajorMinorAxis(0, 2)
                    .BuildMesh();
                return new DualCartesianSimplicialMesh3D(coarseMesh, fineMesh);
            }
        }
    }
}
