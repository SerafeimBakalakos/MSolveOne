using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Interpolation;

namespace MGroup.XFEM.Geometry.LSM.DualMesh
{
    public class DualMeshLsm : IClosedGeometry
    {
        private readonly int dimension;
        private readonly IDualMesh dualMesh;
        private readonly IIsoparametricInterpolation fineMeshInterpolation;
        private readonly ILsmElementInteractionFactory interactionStrategy;
        private readonly ILsmStorage lsmStorage;
        private readonly double intersectionNodeProximityTolerance;

        public DualMeshLsm(int id, IClosedManifold originalGeometry, IDualMesh dualMesh, ILsmStorage lsmStorage,
            double intersectionNodeProximityTolerance = 1E-6)
        {
            this.dualMesh = dualMesh;
            this.ID = id;
            this.intersectionNodeProximityTolerance = intersectionNodeProximityTolerance;

            int dimension = originalGeometry.Dimension;
            if ((dimension != 2) && (dimension != 3))
            {
                throw new ArgumentException("Dimension must be 2 or 3");
            }
            this.dimension = originalGeometry.Dimension;

            if ((originalGeometry.Dimension != dualMesh.Dimension) || (originalGeometry.Dimension != lsmStorage.Dimension))
            {
                throw new ArgumentException("The mesh, original geometry and LSM storage strategy" +
                    " must belong to the spaces with the same dimensionality");
            }
            this.lsmStorage = lsmStorage;

            if (dualMesh.FineMesh.CellType == CellType.Tri3)
            {
                this.interactionStrategy = new LsmTri3Interaction.Factory();
                this.fineMeshInterpolation = InterpolationTri3.UniqueInstance; //TODO: read that from the mesh.
            }
            else if (dualMesh.FineMesh.CellType == CellType.Quad4)
            {
                throw new NotImplementedException();
            }
            else if (dualMesh.FineMesh.CellType == CellType.Tet4)
            {
                this.interactionStrategy = new LsmTet4Interaction.Factory();
                this.fineMeshInterpolation = InterpolationTet4.UniqueInstance;
            }
            else
            {
                throw new NotImplementedException();
            }

            // Initialize the level set values, which may take a while.
            //TODO: perhaps this should be done in a separate method, instead of the constructor.
            lsmStorage.Initialize(originalGeometry, dualMesh.FineMesh);
        }

        public int ID { get; }

        public IElementDiscontinuityInteraction Intersect(IXFiniteElement element)
        {
            // WARNING: This optimization must be avoided. Coarse elements may be flagged as disjoint incorrectly .
            //if (IsCoarseElementDisjoint(element)) return new NullElementDiscontinuityInteraction(this.ID, element);

            bool isIntersected = false;
            var totalIntersectionMesh = new IntersectionMesh(dimension);
            int[] fineElementIDs = dualMesh.MapElementCoarseToFine(element.ID);
            foreach (int fineElementID in fineElementIDs)
            {
                int[] fineElementIdx = dualMesh.FineMesh.GetElementIdx(fineElementID);
                int[] fineElementNodes = dualMesh.FineMesh.GetElementConnectivity(fineElementIdx);

                var nodeCoords = new List<double[]>();
                var nodeLevelSets = new List<double>();
                for (int n = 0; n < fineElementNodes.Length; ++n)
                {
                    nodeCoords.Add(fineMeshInterpolation.NodalNaturalCoordinates[n]);
                    nodeLevelSets.Add(lsmStorage.GetLevelSet(fineElementNodes[n]));
                }

                ILsmCellInteraction interaction = interactionStrategy.CreateNewInteraction(
                    fineElementNodes, nodeCoords, nodeLevelSets, intersectionNodeProximityTolerance);
                RelativePositionCurveElement relativePosition = interaction.Position;
                IntersectionMesh intersectionMesh = interaction.Mesh;

                if ((relativePosition == RelativePositionCurveElement.Disjoint) 
                    || (relativePosition == RelativePositionCurveElement.Tangent))
                {
                    continue;
                }
                if ((relativePosition == RelativePositionCurveElement.Intersecting) 
                    || (relativePosition == RelativePositionCurveElement.Conforming))
                {
                    //TODO: Also take care of the case that the coarse element is conforming. Especially important for comparisons with FEM.
                    //      ow can I check and what to do if the intersection mesh or part of it conforms to the element edges?
                    isIntersected = true;

                    // Convert the coordinates of the intersection points from the natural system of the fine element to the 
                    // natural system of the coarse element.
                    for (int p = 0; p < intersectionMesh.Vertices.Count; ++p)
                    {
                        intersectionMesh.Vertices[p] = dualMesh.MapPointFineNaturalToCoarseNatural(
                            fineElementIdx, intersectionMesh.Vertices[p]);
                    }

                    // Combine the line segments into a mesh
                    totalIntersectionMesh.MergeWith(intersectionMesh, true);
                }
                else
                {
                    throw new NotImplementedException();
                }
            }

            if (isIntersected)
            {
                return InstantiateIntersection(RelativePositionCurveElement.Intersecting, element, totalIntersectionMesh);
            }
            else 
            {
                return new NullElementDiscontinuityInteraction(this.ID, element);
            }
        }

        //TODO: This should be part of the Union() method. But then, the Union would have to check each node all the time, 
        //      even when the client is sure that the 2 geometries can be merged.
        public bool OverlapsWith(IClosedGeometry otherGeometry)
        {
            if (otherGeometry is DualMeshLsm otherLsm)
            {
                if (this.dimension != otherLsm.dimension)
                {
                    throw new ArgumentException("Cannot merge a 2D with a 3D geometry");
                }
                return this.lsmStorage.OverlapsWith(otherLsm.lsmStorage);
            }
            else throw new ArgumentException("Incompatible Level Set geometry");
        }

        public double SignedDistanceOf(XNode node) => lsmStorage.GetLevelSet(dualMesh.MapNodeIDCoarseToFine(node.ID));

        public double SignedDistanceOf(XPoint point)
        {
            int coarseElementID = point.Element.ID;
            double[] coarseNaturalCoords = point.Coordinates[CoordinateSystem.ElementNatural];
            DualMeshPoint dualMeshPoint = dualMesh.CalcShapeFunctions(coarseElementID, coarseNaturalCoords);
            double[] shapeFunctions = dualMeshPoint.FineShapeFunctions;
            int[] fineNodes = dualMesh.FineMesh.GetElementConnectivity(dualMeshPoint.FineElementIdx);

            double result = 0;
            for (int n = 0; n < fineNodes.Length; ++n)
            {
                result += shapeFunctions[n] * lsmStorage.GetLevelSet(fineNodes[n]);
            }
            return result;
        }

        public void UnionWith(IClosedGeometry otherGeometry)
        {
            if (otherGeometry is DualMeshLsm otherLsm)
            {
                if (this.dimension != otherLsm.dimension)
                {
                    throw new ArgumentException("Cannot merge a 2D with a 3D geometry");
                }
                this.lsmStorage.UnionWith(otherLsm.lsmStorage);
            }
            else throw new ArgumentException("Incompatible Level Set geometry");
        }

        private IElementDiscontinuityInteraction InstantiateIntersection(
            RelativePositionCurveElement pos, IXFiniteElement element, IntersectionMesh intersectionMesh)
        {
            if (dimension == 2) return new LsmElementIntersection2D(this.ID, pos, element, intersectionMesh);
            else return new LsmElementIntersection3D(this.ID, pos, element, intersectionMesh);
        }
    }
}
