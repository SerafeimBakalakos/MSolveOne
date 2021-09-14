using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Extensions;

namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// More efficient version of <see cref="ImmersedCrackFront3D"/>. Only adds the bare minimum number of new cells each time 
	/// the crack propagates. Also avoids iterating all the crack mesh entities and only foces on the ones that lie on the crack 
	/// front or are newly added.
	/// </summary>
	public class ImmersedCrackFront3DEfficient : ICrackFront3D
	{
		private readonly CrackSurface3D crackSurface;

		public ImmersedCrackFront3DEfficient(CrackSurface3D crackSurface)
		{
			this.crackSurface = crackSurface;
			Edges = ExtractFrontEdges(crackSurface);

			// The vertices of the boundary are also the start vertex of each edge, since it is a closed polygon.
			Vertices = new List<Vertex3D>(Edges.Count);
			foreach (Edge3D edge in Edges)
			{
				Vertices.Add(edge.Start);
				edge.IsFront = true;
				edge.Start.Position = VertexPosition.TipActive;
			}
			ActiveTips = Enumerable.Range(0, Vertices.Count).ToList();

			// The coordinate systems are determined by the vertices, edges and cells, without enforcing any specific movement.
			CoordinateSystems = new List<CrackFrontSystem3D>();
			for (int v = 0; v < Vertices.Count; ++v)
			{
				Vertex3D current = Vertices[v];
				Vertex3D next = Vertices[(v + 1) % Vertices.Count];
				Vertex3D previous = Vertices[v == 0 ? Vertices.Count - 1 : v - 1];

				var system = new CrackFrontSystem3D(current, previous, next);
				CoordinateSystems.Add(system);
			}
		}

		public List<int> ActiveTips { get; private set; }

		/// <summary>
		/// <see cref="Edges"/>[i] has vertices: start = <see cref="Vertices"/>[i], 
		/// end = <see cref="Vertices"/>[(i+1) % <see cref="Vertices"/>.Count]
		/// </summary>
		public List<Edge3D> Edges { get; }

		public List<CrackFrontSystem3D> CoordinateSystems { get; }

		public List<Vertex3D> Vertices { get; }

		private static List<Edge3D> ExtractFrontEdges(CrackSurface3D crackSurface)
		{
			// Find which edges belong to only 1 cell. These lie on the polyhedron boundary.
			var frontEdgesUnordered = new LinkedList<Edge3D>();
			foreach (Edge3D edge in crackSurface.Edges)
			{
				if (edge.Cells.Count == 1)
				{
					frontEdgesUnordered.AddLast(edge);
				}
			}

			// At these point, the edges have the correct orientation (same as their cell), due to the way they were originally 
			// created. However they must be placed in order.
			var frontEdges = new List<Edge3D>(frontEdgesUnordered.Count);
			frontEdges.Add(frontEdgesUnordered.First.Value); // Process the first edge, does not matter which one
			frontEdgesUnordered.RemoveFirst();
			while (frontEdgesUnordered.Count > 0) // Process the remaining edges. 
			{
				// Find the edge that starts, where the last edge ended.
				Vertex3D currentEnd = frontEdges[frontEdges.Count - 1].End;
				bool exists = frontEdgesUnordered.TryExtract(edge => edge.Start == currentEnd, out Edge3D nextEdge);
				if (!exists)
				{
					throw new ArgumentException("The boundary edges of the polyhedron do not form a closed polygon");
				}
				frontEdges.Add(nextEdge);
			}

			return frontEdges;
		}

		//TODO: split it into smaller methods
		//TODO: this has the side effect that changes the IsFront property of vertices and edges
		public Submesh3D UpdateGeometry(CrackFrontPropagation frontPropagation)
		{
			// Find the coordinates of new vertices
			IList<double[]> newFrontCoords = FindNewFrontCoords(frontPropagation);

			// Create the new crack front vertices
			var newFrontVertices = new List<Vertex3D>(Vertices.Count);
			int numVerticesTotal = crackSurface.Vertices.Count; // excluding crack extension vertices, which will be removed.
			for (int v = 0; v < Vertices.Count; ++v)
			{
				var newVertex = new Vertex3D(numVerticesTotal++, newFrontCoords[v], false);
				newVertex.Position = VertexPosition.TipActive;
				newFrontVertices.Add(newVertex);
			}

			// Create new edges and cells. 
			// This should be done here, since it depends on the concrete class that represents the crack front.
			(List<Edge3D> newEdgesAll, List<Edge3D> newEdgesFront, List<TriangleCell3D> cells) 
				= CreateNewEdgesCells(newFrontVertices);

			// Bidirectional relationships between vertices, edges, cells
			ConnectBidirectionally(newFrontVertices, newEdgesAll, cells);

			// Update coordinate systems at vertices, after determining all connectivity data
			for (int v = 0; v < newFrontVertices.Count; ++v) //TODO duplicate code in constructor
			{
				Vertex3D current = newFrontVertices[v];
				Vertex3D next = newFrontVertices[(v + 1) % newFrontVertices.Count];
				Vertex3D previous = newFrontVertices[v == 0 ? newFrontVertices.Count - 1 : v - 1];
				var system = new CrackFrontSystem3D(current, previous, next);
				CoordinateSystems[v] = system;
			}

			// Replace current front vertices and edges
			for (int v = 0; v < Vertices.Count; ++v)
			{
				Vertices[v].Position = VertexPosition.Internal;
				Vertices[v] = newFrontVertices[v];
			}
			for (int e = 0; e < Edges.Count; ++e)
			{
				Edges[e].IsFront = false;
				Edges[e] = newEdgesFront[e];
			}

			return new Submesh3D() { Vertices = newFrontVertices, Edges = newEdgesAll, Cells = cells };
		}
		
		private (List<Edge3D> newEdgesAll, List<Edge3D> newEdgesFront, List<TriangleCell3D> cells) 
			CreateNewEdgesCells(List<Vertex3D> newFrontVertices)
		{
			// Create new edges and cells. 
			// This should be done here, since it depends on the concrete class that represents the crack front.
			var newCells = new List<TriangleCell3D>(4 * Edges.Count);
			var allNewEdges = new List<Edge3D>(3 * Edges.Count);
			var newFrontEdges = new List<Edge3D>(Edges.Count);
			//var auxiliaryVertices = new List<Vertex3D>();
			for (int v = 0; v < Vertices.Count; ++v)
			{
				Vertex3D vertexOld0 = Vertices[v];
				Vertex3D vertexOld1 = Vertices[(v + 1) % Vertices.Count];
				Vertex3D vertexNew0 = newFrontVertices[v];
				Vertex3D vertexNew1 = newFrontVertices[(v + 1) % Vertices.Count];

				// Cells
				newCells.Add(new TriangleCell3D(vertexOld1, vertexOld0, vertexNew0, false));
				newCells.Add(new TriangleCell3D(vertexNew0, vertexNew1, vertexOld1, false));

				// New front edge
				var frontEdge = new Edge3D(vertexNew0, vertexNew1, false);
				frontEdge.IsFront = true;
				newFrontEdges.Add(frontEdge);
				allNewEdges.Add(frontEdge);

				// Rest of the new edges
				allNewEdges.Add(new Edge3D(vertexOld0, vertexNew0, false));
				//allNewEdges.Add(new Edge3D(vertexOld1, vertexNew1, false)); // This will be created for the next tip
				allNewEdges.Add(new Edge3D(vertexOld1, vertexNew0, false));
			}
			return (/*auxiliaryVertices, */allNewEdges, newFrontEdges, newCells);
		}

		private IList<double[]> FindNewFrontCoords(CrackFrontPropagation frontGrowth)
		{ //TODO: Let the CrackFrontSystem3D do most of these.
			var newFrontVertices = new List<double[]>(Vertices.Count);
			for (int v = 0; v < Vertices.Count; ++v)
			{
				// Params angle and length are given in the coordinate system of each vertex, where the extension vector is 
				// local axis x and the normal vector is local axis y.
				var t = Vector.CreateFromArray(CoordinateSystems[v].Extension);
				var unitT = t.Scale(1.0 / t.Norm2());
				var n = Vector.CreateFromArray(CoordinateSystems[v].Normal); // already unit
				double angle = frontGrowth.AnglesAtTips[v];
				double length = frontGrowth.LengthsAtTips[v];

				// Find the propagation vector in 3D.
				Vector et = (length * Math.Cos(angle)) * unitT;
				Vector en = (length * Math.Sin(angle)) * n;
				Vector propagation = et + en;

				// Find the coordinates of the new vertex
				var oldVertex = Vector.CreateFromArray(Vertices[v].CoordsGlobal);
				Vector newVertex = oldVertex + propagation;
				newFrontVertices.Add(newVertex.RawData);
			}
			return newFrontVertices;
		}

		private void ConnectBidirectionally(IList<Vertex3D> newVertices, IList<Edge3D> newEdges, IList<TriangleCell3D> newCells)
		{
			// Cells of vertices
			foreach (TriangleCell3D cell in newCells)
			{
				foreach (Vertex3D vertex in cell.Vertices)
				{
					vertex.Cells.Add(cell);
				}
			}

			// Edges of vertices
			foreach (Edge3D edge in newEdges)
			{
				edge.Start.Edges.Add(edge);
				edge.End.Edges.Add(edge);
			}

			// Cells of new edges and old front edges
			foreach (Edge3D edge in newEdges.Concat(Edges))
			{
				var commonCells = new HashSet<TriangleCell3D>(edge.Start.Cells);
				commonCells.IntersectWith(edge.End.Cells);
				Debug.Assert((commonCells.Count == 1) || (commonCells.Count == 2));
				edge.Cells.Clear();
				edge.Cells.AddRange(commonCells);
			}

			// Edges of cells
			foreach (TriangleCell3D cell in newCells)
			{
				for (int v = 0; v < cell.Vertices.Length; ++v)
				{
					Vertex3D vertex0 = cell.Vertices[v];
					Vertex3D vertex1 = cell.Vertices[(v + 1) % cell.Vertices.Length];
					var commonEdges = new HashSet<Edge3D>(vertex0.Edges);
					commonEdges.IntersectWith(vertex1.Edges);
					Debug.Assert(commonEdges.Count == 1);
					cell.Edges[v] = commonEdges.First();
				}
			}
		}

		public void Update()
		{
			throw new NotImplementedException();
		}

		public PropagationMesh3D CreatePropagationMesh(CrackFrontPropagation frontPropagation)
		{
			throw new NotImplementedException();
		}
	}
}
