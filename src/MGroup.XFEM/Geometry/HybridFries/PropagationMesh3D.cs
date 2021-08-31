using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

//TODO: Remove repetition between this and CrackSurface3D which handles the existing crack mesh.
namespace MGroup.XFEM.Geometry.HybridFries
{
	public class PropagationMesh3D
	{
		public PropagationMesh3D(List<Vertex3D> frontVertices, List<Edge3D> frontEdges)
		{
			this.FrontVertices = frontVertices;
			this.FrontEdges = frontEdges;

			this.PropagationVertices = new List<Vertex3D>(frontVertices.Count);
			this.PropagationEdges = new List<Edge3D>(frontEdges.Count);
			this.PropagationCells = new List<TriangleCell3D>(4 * frontEdges.Count);
		}

		public List<Edge3D> FrontEdges { get; }

		public List<Vertex3D> FrontVertices { get; }

		public List<TriangleCell3D> PropagationCells { get; } 

		public List<Edge3D> PropagationEdges { get; }

		public List<Vertex3D> PropagationVertices { get; }

		public void CreatePropagationEdges()
		{
			foreach (TriangleCell3D cell in PropagationCells)
			{
				for (int v = 0; v < cell.Vertices.Length; ++v)
				{
					Vertex3D vertex0 = cell.Vertices[v];
					Vertex3D vertex1 = cell.Vertices[(v + 1) % cell.Vertices.Length];

					// Check if this edge is already listed, otherwise create it.
					Edge3D edge = FindEdge(vertex0, vertex1);
					if (edge == null)
					{
						// This makes sure that edges on the boundary of the surface mesh, have the same orientation as the cells
						edge = new Edge3D(vertex0, vertex1);
						PropagationEdges.Add(edge);
					}
					cell.Edges[v] = edge;
				}
			}
		}

		/// <summary>
		/// This will modify the vertices, edges and cells of both the propagation mesh and the existing one.
		/// </summary>
		public void ConnectBidirectionallyVerticesEdgesCells()
		{
			foreach (TriangleCell3D cell in PropagationCells)
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

			foreach (Edge3D edge in PropagationEdges)
			{
				edge.Start.Edges.Add(edge);
				edge.End.Edges.Add(edge);
			}
		}

		/// <summary>
		/// Returns null if there is no edge between the 2 vertices.
		/// </summary>
		/// <param name="vertex0"></param>
		/// <param name="vertex1"></param>
		private Edge3D FindEdge(Vertex3D vertex0, Vertex3D vertex1)
		{
			// Search in propagation edges
			foreach (Edge3D edge in PropagationEdges)
			{
				if (edge.HasVertices(vertex0, vertex1))
				{
					return edge;
				}
			}

			// Search in front edges
			foreach (Edge3D edge in FrontEdges)
			{
				if (edge.HasVertices(vertex0, vertex1))
				{
					return edge;
				}
			}

			// Else
			return null;
		}
	}
}
