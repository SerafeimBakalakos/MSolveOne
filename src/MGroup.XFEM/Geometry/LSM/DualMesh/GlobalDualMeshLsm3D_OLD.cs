using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Geometry.LSM.DualMesh
{
    /// <summary>
    /// Stores level set data in all nodes of the mesh.
    /// </summary>
    public class GlobalDualMeshLsm3D_OLD : DualMeshLsm3DBase_OLD
    {
        private readonly double[] nodalLevelSets;

        public GlobalDualMeshLsm3D_OLD(int id, DualCartesianMesh3D dualMesh, ISurface3D closedSurface) : base(id, dualMesh)
        {
            IStructuredMesh fineMesh = dualMesh.FineMesh;
            nodalLevelSets = new double[fineMesh.NumNodesTotal];
            for (int n = 0; n < nodalLevelSets.Length; ++n)
            {
                double[] node = fineMesh.GetNodeCoordinates(n);
                nodalLevelSets[n] = closedSurface.SignedDistanceOf(node);
            }
        }

        public override void UnionWith(IClosedGeometry otherGeometry)
        {
            if (otherGeometry is GlobalDualMeshLsm3D_OLD otherLsm)
            {
                if (this.nodalLevelSets.Length != otherLsm.nodalLevelSets.Length)
                {
                    throw new ArgumentException("Incompatible Level Set geometry");
                }
                for (int i = 0; i < this.nodalLevelSets.Length; ++i)
                {
                    this.nodalLevelSets[i] = Math.Min(this.nodalLevelSets[i], otherLsm.nodalLevelSets[i]);
                }
            }
            else throw new ArgumentException("Incompatible Level Set geometry");
        }

        protected override double GetLevelSet(int fineNodeID) => nodalLevelSets[fineNodeID];
    }
}
