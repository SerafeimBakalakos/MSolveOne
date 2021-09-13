using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Tolerances;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public class ImplicitElementIntersectionStrategy3D
	{
		private readonly CrackSurface3D crack;

		public ImplicitElementIntersectionStrategy3D(CrackSurface3D crack)
		{
			this.crack = crack;
		}

		public IMeshTolerance MeshTolerance { get; set; } = new ArbitrarySideMeshTolerance();


		public IElementDiscontinuityInteraction FindIntersectionWithCutElement(IXFiniteElement element, bool isTipElement)
		{
			Dictionary<int, double> levelSetSubset = FindLevelSetsOfElementNodes(element);

			ElementFace[] allFaces = element.Faces;
			var intersectionPoints = new Dictionary<double[], HashSet<ElementFace>>();

			// Find any nodes that may lie on the LSM geometry
			for (int n = 0; n < element.Nodes.Count; ++n)
			{
				XNode node = element.Nodes[n];
				double[] nodalLevelSets = crack.GetNodalLevelSets(node);
				if (nodalLevelSets[0] == 0)
				{
					HashSet<ElementFace> facesOfNode = ElementFace.FindFacesOfNode(node.ID, allFaces);
					intersectionPoints.Add(element.Interpolation.NodalNaturalCoordinates[n], facesOfNode);
				}
			}

			// Find intersection points that lie on element edges, excluding nodes
			foreach (ElementEdge edge in element.Edges)
			{
				double[] intersectionPoint = IntersectEdgeExcludingNodes(edge, levelSetSubset);
				if (intersectionPoint != null)
				{
					HashSet<ElementFace> facesOfEdge = edge.FindFacesOfEdge(allFaces);
					intersectionPoints.Add(intersectionPoint, facesOfEdge);
				}
			}

			// Create mesh
			var intersectionMesh = IntersectionMesh3D_OLD.CreateMultiCellMesh3D(intersectionPoints);
			var intersection = new LsmElementIntersection3D(
				crack.ID, RelativePositionCurveElement.Intersecting, element, intersectionMesh);
			intersection.BoundaryOfGeometryInteractsWithElement = isTipElement;
			return intersection;
		}

		private Dictionary<int, double> FindLevelSetsOfElementNodes(IXFiniteElement element)
		{
			var levelSetSubset = new Dictionary<int, double>();
			foreach (XNode node in element.Nodes)
			{
				double[] nodeLevelSets = crack.GetNodalLevelSets(node);
				levelSetSubset[node.ID] = nodeLevelSets[0];
			}
			return levelSetSubset;
		}

		private static double[] IntersectEdgeExcludingNodes(ElementEdge edge, Dictionary<int, double> levelSetSubset)
		{
			double levelSet0 = levelSetSubset[edge.NodeIDs[0]];
			double levelSet1 = levelSetSubset[edge.NodeIDs[1]];
			double[] node0 = edge.NodesNatural[0];
			double[] node1 = edge.NodesNatural[1];

			if (levelSet0 * levelSet1 < 0.0) // Edge is intersected but not at its nodes
			{
				// The intersection point between these nodes can be found using the linear interpolation, see 
				// Sukumar 2001
				double k = -levelSet0 / (levelSet1 - levelSet0);
				var intersection = new double[3];
				for (int d = 0; d < 3; ++d)
				{
					intersection[d] = node0[d] + k * (node1[d] - node0[d]);
				}
				return intersection;
			}
			else return null;
		}
	}
}
