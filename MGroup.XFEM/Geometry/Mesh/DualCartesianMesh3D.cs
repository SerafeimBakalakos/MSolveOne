using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Interpolation;

//TODO: needs extensive testing
namespace MGroup.XFEM.Geometry.Mesh
{
    public class DualCartesianMesh3D : DualCartesianMeshBase
    {
        private DualCartesianMesh3D(UniformCartesianMesh3D coarseMesh, UniformCartesianMesh3D fineMesh) 
            : base(3, coarseMesh, fineMesh)
        {
            CoarseToFineElementOffsets = FindElementOffsets(base.multiple);
        }

        protected override IIsoparametricInterpolation ElementInterpolation => InterpolationHexa8.UniqueInstance;

        protected override List<int[]> CoarseToFineElementOffsets { get; }

        private List<int[]> FindElementOffsets(int[] multiple)
        {
            var elementOffsets = new List<int[]>();
            for (int k = 0; k < multiple[2]; ++k)
            {
                for (int j = 0; j < multiple[1]; ++j)
                {
                    for (int i = 0; i < multiple[0]; ++i)
                    {
                        // Offset from the fine element that has the same first node as the coarse element
                        elementOffsets.Add(new int[] { i, j, k});
                    }
                }
            }
            return elementOffsets;
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

            public DualCartesianMesh3D BuildMesh()
            {
                var coarseMesh = new UniformCartesianMesh3D.Builder(minCoordinates, maxCoordinates, numElementsCoarse)
                    .SetMajorMinorAxis(0, 2) //TODO: Implement the other options in the mesh class and the builder.
                    .BuildMesh();
                var fineMesh = new UniformCartesianMesh3D.Builder(minCoordinates, maxCoordinates, numElementsFine)
                    .SetMajorMinorAxis(0, 2)
                    .BuildMesh();
                return new DualCartesianMesh3D(coarseMesh, fineMesh);
            }
        }
    }
}
