using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Geometry.LSM.DualMesh
{
    public class LsmStorageFixed : LsmStorageLocal
    {
        private readonly List<IClosedManifold> originalGeometries = new List<IClosedManifold>();
        private IStructuredMesh mesh;

        public LsmStorageFixed(int dimension) : base(dimension)
        {
        }

        public override double GetLevelSet(int nodeID)
        {
            bool isNodeNear = this.nodalLevelSets.TryGetValue(nodeID, out double levelSet);
            if (isNodeNear) return levelSet;
            else
            {
                double minDistance = double.MaxValue;
                foreach (IClosedManifold geometry in originalGeometries)
                {
                    double[] coords = mesh.GetNodeCoordinates(nodeID);
                    double distance = geometry.SignedDistanceOf(coords);
                    if (distance < minDistance) minDistance = distance;
                }
                return minDistance;
            }
        }

        public override void Initialize(IClosedManifold originalGeometry, IStructuredMesh mesh)
        {
            base.Initialize(originalGeometry, mesh);
            this.originalGeometries.Add(originalGeometry);
            this.mesh = mesh;
        }


        public override void UnionWith(ILsmStorage otherGeometry)
        {
            if (otherGeometry is LsmStorageFixed casted)
            {
                this.originalGeometries.AddRange(casted.originalGeometries);
            }
            else throw new ArgumentException("Incompatible Level Set geometry");
            base.UnionWith(otherGeometry);
        }
    }
}
