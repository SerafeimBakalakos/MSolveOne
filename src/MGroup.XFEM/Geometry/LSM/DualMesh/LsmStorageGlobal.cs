using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Geometry.LSM.DualMesh
{
    public class LsmStorageGlobal : ILsmStorage
    {
        private double[] nodalLevelSets; //TODO: This assumes the node IDs can be used as indices. Make this a Dictionary
        public LsmStorageGlobal(int dimension)
        {
            this.Dimension = dimension;
        }

        public int Dimension { get; }

        public double GetLevelSet(int nodeID) => nodalLevelSets[nodeID];

        public void Initialize(IClosedManifold originalGeometry, IStructuredMesh mesh)
        {
            if (originalGeometry.Dimension != mesh.Dimension)
            {
                throw new ArgumentException(
                    "The mesh and original geometry must belong to the spaces with the same dimensionality");
            }

            nodalLevelSets = new double[mesh.NumNodesTotal];
            for (int n = 0; n < nodalLevelSets.Length; ++n)
            {
                double[] node = mesh.GetNodeCoordinates(n);
                nodalLevelSets[n] = originalGeometry.SignedDistanceOf(node);
            }
        }

        public bool OverlapsWith(ILsmStorage otherGeometry)
        {
            if (otherGeometry is LsmStorageGlobal casted)
            {
                if (this.Dimension != casted.Dimension)
                {
                    throw new ArgumentException("Cannot merge a 2D with a 3D geometry");
                }
                if (this.nodalLevelSets.Length != casted.nodalLevelSets.Length)
                {
                    throw new ArgumentException("Incompatible Level Set geometry");
                }
                for (int i = 0; i < this.nodalLevelSets.Length; ++i)
                {
                    if ((this.nodalLevelSets[i] <= 0) && (casted.nodalLevelSets[i] <= 0))
                    {
                        return true;
                    }
                }
                return false;
            }
            else throw new ArgumentException("Incompatible Level Set geometry");
        }

        public void UnionWith(ILsmStorage otherGeometry)
        {
            if (otherGeometry is LsmStorageGlobal casted)
            {
                if (this.Dimension != casted.Dimension)
                {
                    throw new ArgumentException("Cannot merge a 2D with a 3D geometry");
                }
                if (this.nodalLevelSets.Length != casted.nodalLevelSets.Length)
                {
                    throw new ArgumentException("Incompatible Level Set geometry");
                }
                for (int i = 0; i < this.nodalLevelSets.Length; ++i)
                {
                    this.nodalLevelSets[i] = Math.Min(this.nodalLevelSets[i], casted.nodalLevelSets[i]);
                }
            }
            else throw new ArgumentException("Incompatible Level Set geometry");
        }
    }
}
