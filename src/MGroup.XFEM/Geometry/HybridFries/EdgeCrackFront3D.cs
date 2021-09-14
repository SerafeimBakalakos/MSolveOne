using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Extensions;
using MGroup.XFEM.Geometry.Boundaries;

namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 3.2.3
	/// </summary>
	public class EdgeCrackFront3D : ICrackFront3D
	{
		private readonly CrackSurface3D crackSurface;
		private readonly IDomainBoundary3D boundary;

		public EdgeCrackFront3D(CrackSurface3D crackSurface, IDomainBoundary3D boundary)
		{
			this.crackSurface = crackSurface;
			this.boundary = boundary;
		}

		public List<int> ActiveTips { get; private set; }

		/// <summary>
		/// <see cref="Edges"/>[i] has vertices: start = <see cref="Vertices"/>[i], 
		/// end = <see cref="Vertices"/>[(i+1) % <see cref="Vertices"/>.Count]
		/// </summary>
		public List<Edge3D> Edges { get; private set; }

		public List<CrackFrontSystem3D> CoordinateSystems { get; private set; }

		public List<Vertex3D> Vertices { get; private set; }

		public PropagationMesh3D CreatePropagationMesh(CrackFrontPropagation frontPropagation)
		{
			var mesh = new PropagationMesh3D(Vertices, Edges);

			// Create the new tips
			int numVerticesTotal = crackSurface.Vertices.Count; // excluding crack extension vertices, which will be removed.
			for (int v = 0; v < Vertices.Count; ++v)
			{
				double[] xOld = Vertices[v].CoordsGlobal;
				double angle = frontPropagation.AnglesAtTips[v];
				double length = frontPropagation.LengthsAtTips[v];
				double[] xNew = CoordinateSystems[v].CalcNewTipCoords(xOld, angle, length);
				var newVertex = new Vertex3D(numVerticesTotal++, xNew, false);
				mesh.PropagationVertices.Add(newVertex);
			}

			// Create the new cells
			for (int v = 0; v < Vertices.Count; ++v)
			{
				Vertex3D vertexOld0 = Vertices[v];
				Vertex3D vertexOld1 = Vertices[(v + 1) % Vertices.Count];
				Vertex3D vertexNew0 = mesh.PropagationVertices[v];
				Vertex3D vertexNew1 = mesh.PropagationVertices[(v + 1) % Vertices.Count];

				// Cells (simplest case, but the front coordinate systems are not very accurate)
				//mesh.PropagationCells.Add(new TriangleCell3D(vertexOld1, vertexOld0, vertexNew0, false));
				//mesh.PropagationCells.Add(new TriangleCell3D(vertexNew0, vertexNew1, vertexOld1, false));

				// Find the centroid and add it to the mesh vertices
				var coords = new double[4][];
				coords[0] = vertexOld0.CoordsGlobal;
				coords[1] = vertexOld1.CoordsGlobal;
				coords[2] = vertexNew0.CoordsGlobal;
				coords[3] = vertexNew1.CoordsGlobal;
				double[] xC = Utilities.FindCentroid((IList<double[]>)coords);
				var centroid = new Vertex3D(numVerticesTotal++, xC, false);
				mesh.PropagationVertices.Add(centroid);

				// 4 triangles between the 5 vertices
				mesh.PropagationCells.Add(new TriangleCell3D(vertexOld1, vertexOld0, centroid, false));
				mesh.PropagationCells.Add(new TriangleCell3D(vertexOld0, vertexNew0, centroid, false));
				mesh.PropagationCells.Add(new TriangleCell3D(vertexNew0, vertexNew1, centroid, false));
				mesh.PropagationCells.Add(new TriangleCell3D(vertexNew1, vertexOld1, centroid, false));
			}

			// Create the new edges 
			mesh.CreatePropagationEdges();
			return mesh;
		}


		public void Update()
		{
			// Clear previous front edges
			if (Edges != null)
			{
				foreach (Edge3D edge in Edges)
				{
					edge.IsFront = false;
				}
			}

			// Find new front edges
			Edges = ExtractFrontEdges();
			foreach (Edge3D edge in Edges)
			{
				edge.IsFront = true;
			}

			// Clear previous tips
			if (Vertices != null)
			{
				foreach (Vertex3D oldTip in Vertices)
				{
					oldTip.Position = VertexPosition.Internal;
				}
			}

			// Find the new tips. Since the crack front is a closed polygon, the tips are the start vertices of each edge.
			// The tips are also the start vertex of each edge, since it is a closed polygon.
			Vertices = new List<Vertex3D>(Edges.Count);
			foreach (Edge3D edge in Edges)
			{
				Vertex3D tip = edge.Start;
				Vertices.Add(tip);
				tip.Position = VertexPosition.TipActive;
			}

			FindActiveTips();

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


		//TODO: Perhaps sort the edges and vertices such that the first vertex is the one with the min ID.
		private List<Edge3D> ExtractFrontEdges()
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

		private void FindActiveTips()
		{
			var positions = new List<RelativePositionManifoldPoint>(Vertices.Count);
			for (int v = 0; v < Vertices.Count; ++v)
			{
				positions.Add(boundary.FindRelativePositionOf(Vertices[v].CoordsGlobal));
			}

			ActiveTips = new List<int>(Vertices.Count);
			for (int v = 0; v < Vertices.Count; ++v)
			{
				if (positions[v] == RelativePositionManifoldPoint.Internal)
				{
					ActiveTips.Add(v);
				}
				else if (positions[v] == RelativePositionManifoldPoint.Boundary)
				{
					int next = (v + 1) % Vertices.Count;
					int previous = v == 0 ? Vertices.Count - 1 : v - 1;
					if ((positions[previous] == RelativePositionManifoldPoint.Internal) 
						|| (positions[next] == RelativePositionManifoldPoint.Internal))
					{
						ActiveTips.Add(v);

						// As the crack propagates, roundoff errors may cause these tips to offset slightly. If this repeats for
						// many crack steps, then they might offset significantly and no longer be recognized as points on the 
						// boundary. Therefore we should reset them to the nearest point on the boundary at each iteration.
						boundary.MinimizeOffsetOfBoundaryPoint(Vertices[v].CoordsGlobal);
					}
				}
				else
				{
					Vertices[v].Position = VertexPosition.TipInactive;

					if (positions[v] == RelativePositionManifoldPoint.External)
					{
						int next = (v + 1) % Vertices.Count;
						int previous = v == 0 ? Vertices.Count - 1 : v - 1;
						if ((positions[previous] == RelativePositionManifoldPoint.Internal)
							|| (positions[next] == RelativePositionManifoldPoint.Internal))
						{
							throw new NotImplementedException(
								"Between an external and an internal tip, there must be a boundary one");
						}
					}
				}
			}
		}
	}
}
