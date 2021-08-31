using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Entities;

//TODO: Also implement the limitations listed in section 3.2.6. However the user should choose if they will be enforced by 
//      stopping the analysis, auto-correcting the crack or just logging the violations. 
namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 3.2
	/// </summary>
	public class CrackSurface3D
	{
		private readonly double maxDomainDimension;
		private readonly bool calcPseudoNormals;
		private Dictionary<int, double[]> nodalLevelSets;

		public CrackSurface3D(int id, double maxDomainDimension, 
			IEnumerable<Vertex3D> vertices, IEnumerable<TriangleCell3D> cells, bool calcPseudoNormals = false)
		{
			ID = id;
			this.maxDomainDimension = maxDomainDimension;
			this.calcPseudoNormals = calcPseudoNormals;
			this.Vertices = new List<Vertex3D>(vertices);
			this.Cells = new List<TriangleCell3D>(cells);
			this.Edges = new List<Edge3D>();
			CreateEdges();
			ConnectVerticesEdgesCells();

			if (calcPseudoNormals)
			{
				//TODO: These could be useful to determine the signs of distances from vertices and edges, 
				//      but not in the way they are implemented.
				CalcPseudoNormals();
			}
		}

		public List<TriangleCell3D> Cells { get; }

		public CrackExtension3D CrackExtension { get; private set; }

		//HERE:
		//TODO: The front should also have an InitializeGeometry() method, instead of doing stuff in the constructor. 
		//      Then it should be injected into the crack surface class's constructor, instead of this property.
		public ICrackFront3D CrackFront { get; set; } 

		public List<Edge3D> Edges { get; }

		public int ID { get; }

		public List<Vertex3D> Vertices { get; }

		public void AlignCells()
		{
			// Change the order of vertices in cells, such that the normal vectors are consistent.
			// This must be done only at the beginning or not at all.
			throw new NotImplementedException();
		}

		public void CalcLevelSets(IXModel model)
		{
			//TODO: This can be sped up significantly by only processing vertices, edges and cells that are inside a spherical 
			//      or cubic bounding box, defined for each node.
			//TODO: This method has a lot of repetition

			this.nodalLevelSets = new Dictionary<int, double[]>();
			foreach (XNode node in model.Nodes.Values)
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
					if (vertex.IsFront && (absDistance < phi2)) phi2 = absDistance;

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
					if (edge.Start.IsExtension && edge.End.IsExtension) continue; // This edge is too far away.

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

				this.nodalLevelSets[node.ID] = new double[] { phi1, phi2, phi3 };
			}
		}

		[Conditional("DEBUG")]
		public void CheckAnglesBetweenCells()
		{
			//for (int c = 0; c < Cells.Count - 1; ++c)
			//{
			//	double[] x0 = Cells[c].Vertices[0].CoordsGlobal;
			//	double[] x1 = Cells[c].Vertices[1].CoordsGlobal;
			//	double[] x2 = Cells[c + 1].Vertices[1].CoordsGlobal;

			//	var v = Vector.CreateFromArray(new double[] { x1[0] - x0[0], x1[1] - x0[1] });
			//	var w = Vector.CreateFromArray(new double[] { x2[0] - x1[0], x2[1] - x1[1] });
			//	double dot = v * w;
			//	if (dot < 0)
			//	{
			//		throw new Exception($"The angle between cells {c} and {c + 1} is not in the [-pi/2, pi/2] range.");
			//	}
			//}
		}

		public double[] GetLevelSetsOf(XNode node) => nodalLevelSets[node.ID];

		public void InitializeGeometry(IXModel model)
		{
			// Explicit description
			this.CrackExtension = new CrackExtension3D(this, maxDomainDimension);

			// Implicit description
			//CalcLevelSets(model);
		}

		public void PropagateCrack(IXModel model, CrackFrontGrowth frontGrowth)
		{
			// Explicit description
			Submesh3D newSubmesh = CrackFront.UpdateGeometry(frontGrowth);
			foreach (Vertex3D vertex in newSubmesh.Vertices) Vertices.Add(vertex);
			foreach (Edge3D edge in newSubmesh.Edges) Edges.Add(edge);
			foreach (TriangleCell3D cell in newSubmesh.Cells) Cells.Add(cell);

			if (calcPseudoNormals)
			{
				CalcPseudoNormals();
			}
			this.CrackExtension = new CrackExtension3D(this, maxDomainDimension);

			// Implicit description
			//CalcLevelSets(model);
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

		private void ConnectVerticesEdgesCells()
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
