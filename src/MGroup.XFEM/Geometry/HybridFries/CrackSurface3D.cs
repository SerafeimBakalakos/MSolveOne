using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Meshes.Manifolds;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 3.2
	/// </summary>
	public class CrackSurface3D : CrackGeometryBase
	{
		private readonly double maxDomainDimension;
		private readonly bool calcPseudoNormals;
		private readonly ImplicitElementIntersectionStrategy3D intersectionStrategy;

		public CrackSurface3D(int id, double maxDomainDimension, 
			IEnumerable<Vertex3D> vertices, IEnumerable<TriangleCell3D> cells/*, bool calcPseudoNormals = false*/)
			: base(id)
		{
			this.maxDomainDimension = maxDomainDimension;
			this.calcPseudoNormals = true/*calcPseudoNormals*/;
			this.intersectionStrategy = new ImplicitElementIntersectionStrategy3D(this);

			this.Vertices = new List<Vertex3D>(vertices);
			this.Cells = new List<TriangleCell3D>(cells);
			this.Edges = new List<Edge3D>();
			CreateEdges();
			ConnectBidirectionallyVerticesEdgesCells();

			if (calcPseudoNormals)
			{
				//TODO: These could be useful to determine the signs of distances from vertices and edges, 
				//      but not in the way they are implemented.
				CalcPseudoNormals();
			}
		}

		public List<TriangleCell3D> Cells { get; }

		public CrackExtension3D CrackExtension { get; private set; }

		//TODO: The front should also have an InitializeGeometry() method, instead of doing stuff in the constructor. 
		//      Then it should be injected into the crack surface class's constructor, instead of this property.
		public ICrackFront3D CrackFront { get; set; } 

		public List<Edge3D> Edges { get; }

		public List<Vertex3D> Vertices { get; }

		public static CrackSurface3D CreateFromMesh(int id, double maxDomainDimension, TriangleMesh3D initialMesh)
		{
			var vertices = new List<Vertex3D>(initialMesh.Vertices.Count);
			for (int v = 0; v < initialMesh.Vertices.Count; ++v)
			{
				vertices.Add(new Vertex3D(v, initialMesh.Vertices[v], false));
			}

			var cells = new List<TriangleCell3D>(initialMesh.Cells.Count);
			for (int c = 0; c < initialMesh.Cells.Count; ++c)
			{
				int[] connectivity = initialMesh.Cells[c];
				Vertex3D v0 = vertices[connectivity[0]];
				Vertex3D v1 = vertices[connectivity[1]];
				Vertex3D v2 = vertices[connectivity[2]];
				cells.Add(new TriangleCell3D(v0, v1, v2, false));
			}

			return new CrackSurface3D(id, maxDomainDimension, vertices, cells);
		}

		public void AlignCells()
		{
			// Change the order of vertices in cells, such that the normal vectors are consistent.
			// This must be done only at the beginning or not at all.
			throw new NotImplementedException();
		}

		public void CalcLevelSets(IEnumerable<XNode> nodes)
		{
			//TODO: This can be sped up significantly by only processing vertices, edges and cells that are inside a spherical 
			//      or cubic bounding box, defined for each node.
			//TODO: This method has a lot of repetition

			this.nodalTripleLevelSets = new Dictionary<int, double[]>();
			foreach (XNode node in nodes)
			{
				double phi1 = double.MaxValue; // unsigned distance to the crack surface, excluding the extension
				double phi2 = double.MaxValue; // unsigned distance to the crack front
				double phi3 = double.NaN; // signed distance to the crack surface, including the extension
				double minAbsPhi3 = double.MaxValue; // the minimum absolute phi3.

				// Process cells of the active crack. 
				foreach (TriangleCell3D cell in Cells)
				{
					double distance = cell.SignedDistanceOf(node.Coordinates);
					if (distance == double.NaN) continue; // The point's projection onto the crack does not lie inside this cell.
					double absDistance = Math.Abs(distance);

					if (absDistance < phi1) phi1 = absDistance;
					if (absDistance < minAbsPhi3)
					{
						minAbsPhi3 = absDistance;
						phi3 = distance;
					}
				}
				//TODO: Can we terminate here without checking the edges and vertices? Perhaps for nice geometries, 
				//      but if the crack curves excessively towards itself then it is not valid.

				// Process edges of the active crack. 
				foreach (Edge3D edge in Edges)
				{
					double absDistance = edge.UnsignedDistanceOf(node.Coordinates);
					if (absDistance == double.NaN) continue; // The point's projection onto the crack does not lie inside this edge.
					int sign = edge.SignOfDistanceOf(node.Coordinates);

					if (absDistance < phi1) phi1 = absDistance;
					if (edge.IsFront && (absDistance < phi2)) phi2 = absDistance;

					// Even if sign = 0, it does not mean that this is the min distance. We must ignore the sign altogether.
					if (absDistance < minAbsPhi3)
					{
						minAbsPhi3 = absDistance;
						phi3 = sign * absDistance;
					}
				}

				// Process vertices of the crack surface, ignoring vertices of the extension, since they are too far away. 
				foreach (Vertex3D vertex in Vertices)
				{
					double absDistance = vertex.UnsignedDistanceOf(node.Coordinates);
					int sign = vertex.SignOfDistanceOf(node.Coordinates);

					if (absDistance < phi1) phi1 = absDistance;
					if ((vertex.Position == VertexPosition.TipActive) && (absDistance < phi2))
					{
						phi2 = absDistance;
					}

					// Even if sign = 0, it does not mean that this is the min distance. We must ignore the sign altogether.
					if (absDistance < minAbsPhi3)
					{
						minAbsPhi3 = absDistance;
						phi3 = sign * absDistance;
					}
				}

				// Process cells of the crack extension. 
				foreach (TriangleCell3D cell in CrackExtension.Cells)
				{
					double distance = cell.SignedDistanceOf(node.Coordinates);
					if (distance == double.NaN) continue; // The point's projection onto the crack does not lie inside this cell.
					double absDistance = Math.Abs(distance);

					if (absDistance < minAbsPhi3)
					{
						minAbsPhi3 = absDistance;
						phi3 = distance;
					}
				}

				// Process edges of the crack extension. 
				// No need to also process vertices of the extension, since they are too far away. 
				foreach (Edge3D edge in CrackExtension.ExtensionEdges)
				{
					if ((edge.Start.Position == VertexPosition.Extension) && (edge.End.Position == VertexPosition.Extension))
					{
						continue; // This edge is too far away.
					}

					double absDistance = edge.UnsignedDistanceOf(node.Coordinates);
					if (absDistance == double.NaN) continue; // The point's projection onto the crack does not lie inside this edge.
					int sign = edge.SignOfDistanceOf(node.Coordinates);

					// Even if sign = 0, it does not mean that this is the min distance. We must ignore the sign altogether.
					if (absDistance < minAbsPhi3)
					{
						minAbsPhi3 = absDistance;
						phi3 = sign * absDistance;
					}
				}

				this.nodalTripleLevelSets[node.ID] = new double[] { phi1, phi2, phi3 };
			}

			base.CalcDoubleLevelSets();
		}

		[Conditional("DEBUG")]
		public void CheckAnglesBetweenCells()
		{
			foreach (Edge3D edge in Edges)
			{
				if (edge.Cells.Count == 1)
				{
					// No angle between cells can be defined at front edges
					continue;
				}

				// The angle between the 2 cells is the complementary of the angle between their normals. 
				// Thus the angle between the normals must be in [-pi/2, pi/2]
				var n1 = Vector.CreateFromArray(edge.Cells[0].Normal);
				var n2 = Vector.CreateFromArray(edge.Cells[1].Normal);
				if (n1 * n2 < 0)
				{
					TriangleCell3D cell0 = edge.Cells[0];
					TriangleCell3D cell1 = edge.Cells[1];
					string cell0Name = $"({cell0.Vertices[0].ID}, {cell0.Vertices[1].ID}, {cell0.Vertices[2].ID})";
					string cell1Name = $"({cell1.Vertices[0].ID}, {cell1.Vertices[1].ID}, {cell1.Vertices[2].ID})";
					throw new Exception(
						$"The angle between cells {cell0Name} and {cell1Name} is not in the [pi/2, 3*pi/2] range.");
				}
			}
		}

		public void InitializeGeometry(IEnumerable<XNode> nodes)
		{
			// Explicit description
			CrackFront.Update();
			CrackExtension = new CrackExtension3D(this, maxDomainDimension);

			// Implicit description
			CalcLevelSets(nodes);
		}

		public override IElementDiscontinuityInteraction Intersect(IXFiniteElement element)
		{
			(RelativePositionCurveElement pos, bool isTipElement) = FindRelativePosition(element);
			if (pos == RelativePositionCurveElement.Disjoint)
			{
				return new NullElementDiscontinuityInteraction(this.ID, element);
			}
			else
			{
				return intersectionStrategy.FindIntersectionWithCutElement(element, isTipElement);
			}
		}

		public void PropagateCrack(IEnumerable<XNode> nodes, CrackFrontPropagation frontGrowth)
		{
			// Explicit description
			PropagationMesh3D mesh = CrackFront.CreatePropagationMesh(frontGrowth);
			foreach (Vertex3D vertex in mesh.PropagationVertices) Vertices.Add(vertex);
			foreach (Edge3D edge in mesh.PropagationEdges) Edges.Add(edge);
			foreach (TriangleCell3D cell in mesh.PropagationCells) Cells.Add(cell);
			mesh.ConnectBidirectionallyVerticesEdgesCells();
			CrackFront.Update();

			if (calcPseudoNormals)
			{
				CalcPseudoNormals();
			}
			this.CrackExtension = new CrackExtension3D(this, maxDomainDimension);

			// Implicit description
			CalcLevelSets(nodes);
		}

		private void CalcPseudoNormals()
		{
			foreach (Vertex3D vertex in Vertices)
			{
				vertex.PseudoNormal = vertex.CalcPseudoNormal();
			}
			foreach (Edge3D edge in Edges)
			{
				edge.PseudoNormal = edge.CalcPseudoNormal();
			}
		}

		private void ConnectBidirectionallyVerticesEdgesCells()
		{
			foreach (TriangleCell3D cell in Cells)
			{
				foreach (Vertex3D vertex in cell.Vertices)
				{
					vertex.Cells.Add(cell);
				}

				foreach (Edge3D edge in cell.Edges)
				{
					edge.Cells.Add(cell);
					Debug.Assert(edge.Cells.Count <= 2);
				}
			}

			foreach (Edge3D edge in Edges)
			{
				edge.Start.Edges.Add(edge);
				edge.End.Edges.Add(edge);
			}
		}

		private void CreateEdges()
		{
			Edges.Clear();
			foreach (TriangleCell3D cell in Cells)
			{
				for (int v = 0; v < cell.Vertices.Length; ++v)
				{
					Vertex3D vertex0 = cell.Vertices[v];
					Vertex3D vertex1 = cell.Vertices[(v + 1) % cell.Vertices.Length];

					// Check if this edge is already listed, otherwise create it.
					Edge3D thisEdge = null;
					foreach (Edge3D otherEdge in Edges)
					{
						if (otherEdge.HasVertices(vertex0, vertex1))
						{
							thisEdge = otherEdge;
							break;
						}
					}
					if (thisEdge == null)
					{
						// This makes sure that edges on the boundary of the surface mesh, have the same orientation as the cells
						thisEdge = new Edge3D(vertex0, vertex1);
						Edges.Add(thisEdge);
					}
					cell.Edges[v] = thisEdge;
				}
			}
		}
	}
}
