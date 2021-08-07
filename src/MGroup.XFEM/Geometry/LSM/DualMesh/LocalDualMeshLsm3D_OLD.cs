using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Geometry.Primitives;

//TODO: perhaps instead of using a Dictionary<int, double> for the level sets, I could store them based on the node index:
//      Dictionary<int, Dictionary<int, double>>. The aim is of course faster lookups and faster initialization.
namespace MGroup.XFEM.Geometry.LSM.DualMesh
{
    /// <summary>
    /// Only stores level set data in nodes that are inside the curve or belong to elements that are intersected by it.
    /// Therefore the signed distances of points away from the curve will not be accurate, but will have the correct sign.
    /// </summary>
    public class LocalDualMeshLsm3D_OLD : DualMeshLsm3DBase_OLD
    {
        private readonly double farNodeDistance;
        protected readonly Dictionary<int, double> nodalLevelSets;

        public LocalDualMeshLsm3D_OLD(int id, DualCartesianMesh3D dualMesh, ISurface3D closedSurface) : base(id, dualMesh)
        {
            // Only keep the level sets of elements that are inside the initial geometry, intersected by it or conforming to it.
            IStructuredMesh fineMesh = dualMesh.FineMesh;
            nodalLevelSets = new Dictionary<int, double>();
            double maxLevelSet = double.MinValue; // over all nodes in the fine mesh
            for (int e = 0; e < fineMesh.NumElementsTotal; ++e)
            {
                // Find the level sets at the nodes of this element
                int[] nodeIDs = fineMesh.GetElementConnectivity(e);
                var levelSets = new double[nodeIDs.Length];
                for (int n = 0; n < nodeIDs.Length; ++n)
                {
                    int nodeID = nodeIDs[n];
                    double[] coords = fineMesh.GetNodeCoordinates(nodeID);
                    levelSets[n] = closedSurface.SignedDistanceOf(coords);
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


        public override void UnionWith(IClosedGeometry otherGeometry)
        {
            if (otherGeometry is LocalDualMeshLsm3D_OLD otherLsm)
            {
                foreach (var nodeValuePair in otherLsm.nodalLevelSets)
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

        protected override double GetLevelSet(int fineNodeID)
        {
            bool isNodeNear = this.nodalLevelSets.TryGetValue(fineNodeID, out double levelSet);
            if (isNodeNear) return levelSet;
            else return farNodeDistance;
        }
    }
}
