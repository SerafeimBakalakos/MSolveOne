using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Interpolation;

//TODO: Duplication between this and the nonsymmetric version. Use a dedicated ILevelSetStorage for this one and a base class for
//		the signed distances.
namespace MGroup.XFEM.Geometry.LSM.DualMesh
{
	public class DualMeshLsmSymmetric_OLD : IClosedGeometry
	{
		private readonly int dimension;
		private readonly DualCartesianSimplicialSymmetricMeshBase_OLD dualMesh;
		private readonly IIsoparametricInterpolation fineMeshInterpolation;
		private readonly ILsmElementInteractionFactory interactionStrategy;
		private readonly double intersectionNodeProximityTolerance;
		private readonly double[] levelSetsCoarseMesh;
		private readonly double[] levelSetsFineMesh;

		public DualMeshLsmSymmetric_OLD(int id, IClosedManifold originalGeometry, DualCartesianSimplicialSymmetricMeshBase_OLD dualMesh,
			double intersectionNodeProximityTolerance = 1E-6)
		{
			this.dualMesh = dualMesh;
			this.ID = id;
			this.intersectionNodeProximityTolerance = intersectionNodeProximityTolerance;

			if ((dualMesh.Dimension != 2) && (dualMesh.Dimension != 3))
			{
				throw new ArgumentException("Mesh dimension must be 2 or 3");
			}
			this.dimension = dualMesh.Dimension;

			if (originalGeometry.Dimension != dualMesh.Dimension)
			{
				throw new ArgumentException("The mesh and original geometry" +
					" must belong to the spaces with the same dimensionality");
			}

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

			// Initialize the level set values at the nodes of the fine and coarse mesh, which may take a while.
			//TODO: perhaps this should be done in a separate method, instead of the constructor.
			levelSetsCoarseMesh = InitializeLevelSets(originalGeometry, dualMesh.CoarseMesh);
			//levelSetsFineMesh = InitializeLevelSets(originalGeometry, dualMesh.FineMesh);
			levelSetsFineMesh = InitializeLevelSetsFineMesh(levelSetsCoarseMesh, dualMesh);
		}

		public DualMeshLsmSymmetric_OLD(int id, double[] levelSetsCoarseMesh, DualCartesianSimplicialSymmetricMeshBase_OLD dualMesh,
			double intersectionNodeProximityTolerance = 1E-6)
		{
			this.dualMesh = dualMesh;
			this.ID = id;
			this.intersectionNodeProximityTolerance = intersectionNodeProximityTolerance;

			if ((dualMesh.Dimension != 2) && (dualMesh.Dimension != 3))
			{
				throw new ArgumentException("Mesh dimension must be 2 or 3");
			}
			this.dimension = dualMesh.Dimension;

			if (levelSetsCoarseMesh.Length != dualMesh.CoarseMesh.NumNodesTotal)
			{
				throw new ArgumentException($"The coarse mesh has {dualMesh.CoarseMesh.NumNodesTotal} nodes," +
					$" but {levelSetsCoarseMesh.Length} level set values were provided.");
			}
			this.levelSetsCoarseMesh = levelSetsCoarseMesh;

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

			// Initialize the level set values at the nodes of the fine mesh, which may take a while.
			//TODO: perhaps this should be done in a separate method, instead of the constructor.
			levelSetsFineMesh = InitializeLevelSetsFineMesh(levelSetsCoarseMesh, dualMesh);
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
					nodeLevelSets.Add(levelSetsFineMesh[fineElementNodes[n]]);
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

		public bool OverlapsWith(IClosedGeometry otherGeometry)
		{
			throw new NotImplementedException();

		}

		public double SignedDistanceOf(XNode node) => levelSetsCoarseMesh[node.ID];

		public double SignedDistanceOf(XPoint point)
		{
			int coarseElementID = point.Element.ID;
			int[] coarseElementIdx = dualMesh.CoarseMesh.GetElementIdx(coarseElementID);
			int[] coarseNodes = dualMesh.CoarseMesh.GetElementConnectivity(coarseElementIdx);
			if (point.ShapeFunctions == null)
			{
				double[] coarseNaturalCoords = point.Coordinates[CoordinateSystem.ElementNatural];
				point.ShapeFunctions = dualMesh.CoarseElementInterpolation.EvaluateFunctionsAt(coarseNaturalCoords);
			}
			double[] shapeFunctions = point.ShapeFunctions;

			double result = 0;
			for (int n = 0; n < coarseNodes.Length; ++n)
			{
				result += shapeFunctions[n] * levelSetsCoarseMesh[coarseNodes[n]];
			}
			return result;
		}

		public void UnionWith(IClosedGeometry otherGeometry)
		{
			throw new NotImplementedException();
		}

		private static double[] InitializeLevelSets(IClosedManifold originalGeometry,
			IStructuredMesh mesh)
		{
			var nodalLevelSets = new double[mesh.NumNodesTotal];
			for (int n = 0; n < nodalLevelSets.Length; ++n)
			{
				double[] node = mesh.GetNodeCoordinates(n);
				nodalLevelSets[n] = originalGeometry.SignedDistanceOf(node);
			}
			return nodalLevelSets;
		}

		private static double[] InitializeLevelSetsFineMesh(double[] levelSetsCoarseMesh, 
			DualCartesianSimplicialSymmetricMeshBase_OLD dualMesh)
		{
			var levelSetsFineMesh = new double[dualMesh.FineMesh.NumNodesTotal];
			for (int fineNodeID = 0; fineNodeID < dualMesh.FineMesh.NumNodesTotal; ++fineNodeID) //TODO: The fine mesh node IDs might not be contiguous
			{
				// Interpolate the level sets of surrounding coarse nodes
				int[] surroundingCoarseNodeIDs = dualMesh.FindSurroundingCoarseNodesForFineNode(fineNodeID);
				double fineNodeLevelSet = 0;
				foreach (int coarseNodeID in surroundingCoarseNodeIDs)
				{
					fineNodeLevelSet += levelSetsCoarseMesh[coarseNodeID];
				}
				fineNodeLevelSet /= surroundingCoarseNodeIDs.Length;
				levelSetsFineMesh[fineNodeID] = fineNodeLevelSet;
			}
			return levelSetsFineMesh;
		}

		private IElementDiscontinuityInteraction InstantiateIntersection(
			RelativePositionCurveElement pos, IXFiniteElement element, IntersectionMesh intersectionMesh)
		{
			if (dimension == 2) return new LsmElementIntersection2D(this.ID, pos, element, intersectionMesh);
			else return new LsmElementIntersection3D(this.ID, pos, element, intersectionMesh);
		}
	}
}
