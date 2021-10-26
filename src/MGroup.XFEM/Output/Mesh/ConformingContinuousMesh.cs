using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Commons;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Mesh
{
	/// <summary>
	/// It conforms to discontinuities by using intersection points as vertices, in addition to the nodes. Nodes that belong to 
	/// multiple elements are unique and can be used to smooth the fields over them. Intersection points that belong to multiple
	/// elements will be present twice; once for each side of the discontinuity.
	/// /// </summary>
	public class ConformingContinuousMesh : IOutputMesh
	{
		/// <summary>
		/// Intersection points will be moved along their edge by <see cref="offsetPercentage"/> * length(edge).
		/// </summary>
		private const double offsetPercentage = 1E-6;

		/// <summary>
		/// An intesection point belongs to an edge if 
		/// distance(point, edge) &lt; <see cref="edgeProximityTolerance"/> * length(edge).
		/// </summary>
		private const double edgeProximityTolerance = 1E-6;

		private static readonly ValueComparer pointComparer = new ValueComparer(1E-6);

		private readonly IXModel model;

		public ConformingContinuousMesh(IXModel model, IXGeometryDescription discontinuity)
		{
			this.model = model;
			this.Discontinuity = discontinuity;
			ProcessOriginalNodes(model);

			// Process intersection points
			foreach (IXFiniteElement element in model.EnumerateElements())
			{
				if ((element.ConformingSubcells != null) && (element.ConformingSubcells.Length > 0))
				{
					ProcessElementIntersected(element);
				}
				else
				{
					ProcessElementStandard(element);
				}
			}
			OffsetIntersectionVertices();
			IntersectionPoints.Clear();
		}

		public List<VtkCell> Cells { get; } = new List<VtkCell>();

		public IXGeometryDescription Discontinuity { get; }

		public Dictionary<int, VtkPoint> Vertices { get; } = new Dictionary<int, VtkPoint>();

		public Dictionary<int, VertexCoordinates> VerticesCoordinates { get; } = new Dictionary<int, VertexCoordinates>();

		private List<IntersectionPoint> IntersectionPoints { get; } = new List<IntersectionPoint>();

		public int NumOutCells => Cells.Count;

		public int NumOutVertices => Vertices.Count;

		public IEnumerable<VtkCell> OutCells => Cells;

		public IEnumerable<VtkPoint> OutVertices => Vertices.OrderBy(pair => pair.Key).Select(pair => pair.Value);

		private static bool ArePointsEqual(double[] pointA, double[] pointB, ValueComparer comparer = null)
		{
			if (pointA.Length != pointB.Length)
			{
				throw new ArgumentException("Both points must have the same dimensions");
			}
			if (comparer != null)
			{
				for (int d = 0; d < pointA.Length; ++d)
				{
					if (!comparer.AreEqual(pointA[d], pointB[d]))
					{
						return false;
					}
				}
			}
			else
			{
				for (int d = 0; d < pointA.Length; ++d)
				{
					if (pointA[d] != pointB[d])
					{
						return false;
					}
				}
			}

			return true;
		}

		private static double[] CopyArray(double[] original)
		{
			var copy = new double[original.Length];
			for (int i = 0; i < original.Length; ++i)
			{
				copy[i] = original[i];
			}
			return copy;
		}

		private static ElementEdge FindEdgeOfIntersectionPoint(IXFiniteElement element, double[] pointNatural)
		{
			var nearbyEdges = new Dictionary<int, double>();
			for (int e = 0; e < element.Edges.Length; ++e)
			{
				ElementEdge edge = element.Edges[e];
				Vector pointA, pointB;
				if (edge.NodeIDs[0] < edge.NodeIDs[1])
				{ // This will ensure that the same order is followed for all elements with this edge
					pointA = Vector.CreateFromArray(edge.NodesNatural[0]);
					pointB = Vector.CreateFromArray(edge.NodesNatural[1]);
				}
				else
				{
					pointA = Vector.CreateFromArray(edge.NodesNatural[1]);
					pointB = Vector.CreateFromArray(edge.NodesNatural[0]);
				}
				var pointC = Vector.CreateFromArray(pointNatural);

				Vector vAB = pointB - pointA;
				double edgeLength = vAB.Norm2();
				Vector vAC = pointC - pointA;
				Vector projection = ((vAC * vAB) / (edgeLength * edgeLength)) * vAB;
				Vector rejection = vAC - projection;
				double distance = rejection.Norm2();

				if (distance <= edgeProximityTolerance * edgeLength)
				{
					nearbyEdges[e] = distance;
				}
			}

			if (nearbyEdges.Count == 0)
			{
				throw new NotImplementedException(
					"This intersection point lies on the interior of the element instead of an edge");
			}
			else if (nearbyEdges.Count == 1)
			{
				return element.Edges[nearbyEdges.Keys.First()];
			}
			else
			{
				// If the element is very close to a node, then it may be flagged as belonging to multiple edges. 
				// Keep the closest.
				double minDistance = double.MaxValue;
				int closestEdgeIdx = -1;
				foreach (var idxDistancePair in nearbyEdges)
				{
					double distance = idxDistancePair.Value;
					if (distance < minDistance)
					{
						minDistance = distance;
						closestEdgeIdx = idxDistancePair.Key;
					}
				}
				return element.Edges[closestEdgeIdx];
			}
		}

		private static double[] FindNaturalCoordsOfNode(int nodeID, IXFiniteElement element)
		{
			for (int n = 0; n < element.Nodes.Count; ++n)
			{
				if (element.Nodes[n].ID == nodeID)
				{
					return element.Interpolation.NodalNaturalCoordinates[n];
				}
			}
			throw new ArgumentException($"Node {nodeID} does not belong to element {element.ID}");
		}

		/// <summary>
		/// Modifies <paramref name="pointP"/>, such that P = P + (e * |AB|) * PA / |PA|, where e = <see cref="offsetPercentage"/>.
		/// </summary>
		/// <param name="pointP">Will be overwritten by the result.</param>
		/// <param name="nodeA">The node that lies on the same side as <paramref name="pointP"/>.</param>
		/// <param name="nodeB"></param>
		private static void OffsetPoint(double[] pointP, double[] nodeA, double[] nodeB)
		{
			Vector pP = Vector.CreateFromArray(pointP, false);
			Vector pA = Vector.CreateFromArray(nodeA, true);
			Vector pB = Vector.CreateFromArray(nodeB, true);
			double step = offsetPercentage * (pB - pA).Norm2();
			Vector vPA = pA - pP;
			Vector offset = (step / vPA.Norm2()) * vPA;
			pP.AddIntoThis(offset);
		}

		private int CreateIntersectionVertex(IXFiniteElement element, IElementSubcell subcell, int pointIdx, bool positiveSide)
		{
			// Find the edge it belongs to
			double[] pointNatural = subcell.VerticesNatural[pointIdx];
			ElementEdge edge = FindEdgeOfIntersectionPoint(element, pointNatural);
			var intersectionPoint = new IntersectionPoint(element, pointNatural, edge, positiveSide);

			// Add it to the total intersections points, if it is not a duplicate.
			int vertexID = int.MinValue;
			foreach (IntersectionPoint existingPoint in IntersectionPoints)
			{
				if (existingPoint.Equals(intersectionPoint))
				{
					// Combine the data of the existing point with its new duplicate
					vertexID = existingPoint.VertexID;
					existingPoint.CopyNaturalCoordsFrom(intersectionPoint);
					break;
				}
			}

			if (vertexID == int.MinValue)
			{
				// Find the vertex id of this point and the opposite one (same coords, opposite side of discontinuity).
				int oppositeID;
				if (!intersectionPoint.PositiveSide)
				{
					vertexID = Vertices.Count;
					oppositeID = vertexID + 1;
				}
				else
				{
					oppositeID = Vertices.Count;
					vertexID = oppositeID + 1;
				}

				// Add this point to the total vertices and intersection points.
				intersectionPoint.VertexID = vertexID;
				Vertices[vertexID] = new VtkPoint(vertexID, intersectionPoint.CoordsGlobal);
				IntersectionPoints.Add(intersectionPoint);

				// Repeat for the opposite point
				double[] coordsOpposite = CopyArray(intersectionPoint.CoordsGlobal);
				var pointOpposite = new IntersectionPoint(
					element, pointNatural, edge, !intersectionPoint.PositiveSide, coordsOpposite);
				pointOpposite.VertexID = oppositeID;
				IntersectionPoints.Add(pointOpposite);
				Vertices[oppositeID] = new VtkPoint(oppositeID, coordsOpposite);
			}

			return vertexID;
		}

		private void ProcessOriginalNodes(IXModel model)
		{
			foreach (XNode node in model.Nodes.Values)
			{
				int id = node.ID;
				Vertices[id] = new VtkPoint(id, node.Coordinates);
				var vertexCoords = new VertexCoordinates();
				vertexCoords.OriginalNodeID = id;
				foreach (IXFiniteElement element in node.ElementsDictionary.Values)
				{
					int idx = element.Nodes.ToList().IndexOf(node);
					double[] naturalCoords = element.Interpolation.NodalNaturalCoordinates[idx];
					vertexCoords.NaturalCoords[element] = naturalCoords;
				}
				VerticesCoordinates[id] = vertexCoords;
			}
		}

		//TODO: this does not work if the nodes lie on the interfaces
		private void ProcessElementIntersected(IXFiniteElement element)
		{
			var nodesNatural = new List<double[]>(element.Interpolation.NodalNaturalCoordinates);
			foreach (IElementSubcell subcell in element.ConformingSubcells)
			{
				var centroid = new XPoint(model.Dimension);
				centroid.Element = element;
				double[] centroidCoords = subcell.FindCentroidNatural();
				centroid.Coordinates[CoordinateSystem.ElementNatural] = centroidCoords;
				centroid.ShapeFunctions = element.Interpolation.EvaluateFunctionsAt(centroidCoords);
				double distance = Discontinuity.SignedDistanceOf(centroid);
				bool positiveSide = distance > 0;

				int numVertices = subcell.VerticesNatural.Count;
				var outvertices = new VtkPoint[numVertices];
				for (int i = 0; i < numVertices; ++i)
				{
					double[] vertexNatural = subcell.VerticesNatural[i];
					int nodeIdx = nodesNatural.FindIndex(node => ArePointsEqual(node, vertexNatural));
					if (nodeIdx >= 0) // This point coincides with a node
					{
						// Nodes are already processed at this point. We only need to associate the vertex with that node.
						outvertices[i] = Vertices[element.Nodes[nodeIdx].ID];
					}
					else
					{
						int vertexID = CreateIntersectionVertex(element, subcell, i, positiveSide);
						outvertices[i] = Vertices[vertexID];
					}
				}

				var cell = new VtkCell(subcell.CellType, outvertices);
				Cells.Add(cell);
			}
		}

		private void ProcessElementStandard(IXFiniteElement element)
		{
			VtkPoint[] outVertices = element.Nodes.Select(node => Vertices[node.ID]).ToArray();
			var cell = new VtkCell(element.CellType, outVertices);
			Cells.Add(cell);
		}

		private void OffsetIntersectionVertices()
		{
			foreach (IntersectionPoint point in IntersectionPoints)
			{
				// Find the node that lies on the same side as the point
				(XNode nodeSameSide, XNode nodeOppositeSide) = point.FindNodeAtEachSide(model, Discontinuity);

				// Offset global coordinates
				OffsetPoint(Vertices[point.VertexID].Coordinates, nodeSameSide.Coordinates, nodeOppositeSide.Coordinates);

				// Offset natural coordinates and store them
				var vertexCoords = new VertexCoordinates();
				foreach (IXFiniteElement element in point.CoordsNatural.Keys)
				{
					double[] pointNatural = point.CoordsNatural[element];
					double[] nodeANatural = FindNaturalCoordsOfNode(nodeSameSide.ID, element);
					double[] nodeBNatural = FindNaturalCoordsOfNode(nodeOppositeSide.ID, element);
					OffsetPoint(pointNatural, nodeANatural, nodeBNatural);
					vertexCoords.NaturalCoords[element] = pointNatural;
				}
				VerticesCoordinates[point.VertexID] = vertexCoords;
			}
		}

		public class VertexCoordinates
		{
			public VertexCoordinates() 
			{
				NaturalCoords = new Dictionary<IXFiniteElement, double[]>();
			}

			public VertexCoordinates(Dictionary<IXFiniteElement, double[]> naturalCoords) 
			{
				this.NaturalCoords = new Dictionary<IXFiniteElement, double[]>(naturalCoords);
			}

			public Dictionary<IXFiniteElement, double[]> NaturalCoords { get; }

			public int OriginalNodeID { get; set; } = int.MinValue;
		}

		public class IntersectionPoint
		{
			public IntersectionPoint(IXFiniteElement element, double[] coordsNatural, ElementEdge edge, bool positiveSide, 
				double[] coordsGlobal = null)
			{
				CoordsNatural = new Dictionary<IXFiniteElement, double[]>();
				CoordsNatural[element] = coordsNatural;

				if (coordsGlobal != null)
				{
					this.CoordsGlobal = coordsGlobal;
				}
				else
				{
					this.CoordsGlobal = element.Interpolation.TransformNaturalToCartesian(element.Nodes, coordsNatural);
				}

				if (edge.NodeIDs[0] < edge.NodeIDs[1])
				{
					EdgeStart = edge.NodeIDs[0];
					EdgeEnd = edge.NodeIDs[1];
				}
				else
				{
					EdgeStart = edge.NodeIDs[1];
					EdgeEnd = edge.NodeIDs[0];
				}

				this.PositiveSide = positiveSide;
			}

			public double[] CoordsGlobal { get; }

			public Dictionary<IXFiniteElement, double[]> CoordsNatural { get; } 

			/// <summary>
			/// The min id of the edge's nodes.
			/// </summary>
			public int EdgeStart { get; }

			/// <summary>
			/// The max id of the edge's nodes.
			/// </summary>
			public int EdgeEnd { get; }

			/// <summary>
			/// The edge node that lies on the same side of the discontinuity as the intersection point.
			/// </summary>
			public bool PositiveSide { get; }

			public int VertexID { get; set; }

			public void CopyNaturalCoordsFrom(IntersectionPoint other)
			{
				foreach (var pair in other.CoordsNatural)
				{
					IXFiniteElement element = pair.Key;
					double[] coords = pair.Value;
					this.CoordsNatural[element] = coords; //TODO: If this element (key) already exist, make sure tha e coordinates are the same.
				}
			}

			public bool Equals(IntersectionPoint other)
			{
				if (this.EdgeStart != other.EdgeStart)
				{
					return false;
				}
				if (this.EdgeEnd != other.EdgeEnd)
				{
					return false;
				}
				if (this.PositiveSide != other.PositiveSide)
				{
					return false;
				}

				return ArePointsEqual(this.CoordsGlobal, other.CoordsGlobal, pointComparer);
			}

			public (XNode nodeSameSide, XNode nodeOppositeSide) FindNodeAtEachSide(
				IXModel model, IXGeometryDescription discontinuity)
			{
				XNode nodeStart = model.Nodes[this.EdgeStart];
				XNode nodeEnd = model.Nodes[this.EdgeEnd];
				double signPoint = this.PositiveSide ? 1 : -1;
				double signNodeStart = discontinuity.SignedDistanceOf(nodeStart);
				XNode nodeSameSide, nodeOppositeSide;
				if (signPoint * signNodeStart > 0)
				{
					return (nodeStart, nodeEnd);
				}
				else
				{
					return (nodeEnd, nodeStart);
				}
			}
		}
	}
}
