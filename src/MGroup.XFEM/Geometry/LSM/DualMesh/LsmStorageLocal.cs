using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Geometry.LSM.DualMesh
{
    public class LsmStorageLocal : ILsmStorage
    {
        private double farNodeDistance;
        protected Dictionary<int, double> nodalLevelSets;

        public LsmStorageLocal(int dimension)
        {
            this.Dimension = dimension;
        }

        public int Dimension { get; }

        public virtual double GetLevelSet(int nodeID)
        {
            bool isNodeNear = this.nodalLevelSets.TryGetValue(nodeID, out double levelSet);
            if (isNodeNear) return levelSet;
            else return farNodeDistance;
        }

        public bool OverlapsWith(ILsmStorage otherGeometry)
        {
            if (otherGeometry is LsmStorageLocal casted)
            {
                if (this.Dimension != casted.Dimension)
                {
                    throw new ArgumentException("Cannot merge a 2D with a 3D geometry");
                }

                foreach (var nodeValuePair in casted.nodalLevelSets)
                {
                    int nodeID = nodeValuePair.Key;
                    double otherLevelSet = nodeValuePair.Value;
                    bool isCommonNode = this.nodalLevelSets.TryGetValue(nodeID, out double thisLevelSet);
                    if (isCommonNode)
                    {
                        return true;
                    }
                }
                return false;
            }
            else throw new ArgumentException("Incompatible Level Set geometry");
        }

        public virtual void Initialize(IClosedManifold originalGeometry, IStructuredMesh mesh)
        {
            if (originalGeometry.Dimension != mesh.Dimension)
            {
                throw new ArgumentException(
                    "The mesh and original geometry must belong to the spaces with the same dimensionality");
            }

            // Only keep the level sets of elements that are inside the initial geometry, intersected by it or conforming to it.
            nodalLevelSets = new Dictionary<int, double>();
            double maxLevelSet = double.MinValue; // over all nodes in the fine mesh
            for (int e = 0; e < mesh.NumElementsTotal; ++e)
            {
                // Find the level sets at the nodes of this element
                int[] nodeIDs = mesh.GetElementConnectivity(e);
                var levelSets = new double[nodeIDs.Length];
                for (int n = 0; n < nodeIDs.Length; ++n)
                {
                    int nodeID = nodeIDs[n];
                    double[] coords = mesh.GetNodeCoordinates(nodeID);
                    levelSets[n] = originalGeometry.SignedDistanceOf(coords);
                }

                // If even one node is inside or on the curve, we need to store the level sets for all nodes of the element
                double minLevelSet = double.MaxValue;
                for (int n = 0; n < nodeIDs.Length; ++n)
                {
                    double levelSet = levelSets[n];
                    if (levelSet < minLevelSet) minLevelSet = levelSet;
                    if (levelSet > maxLevelSet) maxLevelSet = levelSet;
                }

                if (minLevelSet <= 0.0)
                {
                    for (int n = 0; n < nodeIDs.Length; ++n)
                    {
                        int nodeID = nodeIDs[n];
                        nodalLevelSets[nodeID] = levelSets[n];
                    }
                }
            }
            farNodeDistance = maxLevelSet;
        }

        public virtual void UnionWith(ILsmStorage otherGeometry)
        {
            if (otherGeometry is LsmStorageLocal casted)
            {
                if (this.Dimension != casted.Dimension)
                {
                    throw new ArgumentException("Cannot merge a 2D with a 3D geometry");
                }

                foreach (var nodeValuePair in casted.nodalLevelSets)
                {
                    int nodeID = nodeValuePair.Key;
                    double otherLevelSet = nodeValuePair.Value;
                    bool isCommonNode = this.nodalLevelSets.TryGetValue(nodeID, out double thisLevelSet);
                    if (isCommonNode)
                    {
                        this.nodalLevelSets[nodeID] = Math.Min(thisLevelSet, otherLevelSet);
                    }
                    else
                    {
                        this.nodalLevelSets[nodeID] = otherLevelSet;
                    }
                }
            }
            else throw new ArgumentException("Incompatible Level Set geometry");
        }
    }
}
