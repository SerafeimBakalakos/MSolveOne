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
    public class GlobalDualMeshLsm2D_OLD : DualMeshLsm2DBase_OLD
    {
        private readonly double[] nodalLevelSets;

        public GlobalDualMeshLsm2D_OLD(int id, IDualMesh dualMesh, ICurve2D closedCurve) : base(id, dualMesh)
        {
            IStructuredMesh fineMesh = dualMesh.FineMesh;
            nodalLevelSets = new double[fineMesh.NumNodesTotal];
            for (int n = 0; n < nodalLevelSets.Length; ++n)
            {
                double[] node = fineMesh.GetNodeCoordinates(n);
                nodalLevelSets[n] = closedCurve.SignedDistanceOf(node);
            }
        }

        public override void UnionWith(IClosedGeometry otherGeometry)
        {
            if (otherGeometry is GlobalDualMeshLsm2D_OLD otherLsm)
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
