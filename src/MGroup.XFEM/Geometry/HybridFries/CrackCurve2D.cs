using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Entities;

//TODO: Also implement the limitations listed in section 3.1.6. However the user should choose if they will be enforced by 
//      stopping the analysis, auto-correcting the crack or just logging the violations. 
namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 3.1
	/// </summary>
	public class CrackCurve2D
	{
		private readonly double maxDomainDimension;
		private readonly bool calcPseudoNormals;
		private Dictionary<int, double[]> nodalLevelSets;

		public CrackCurve2D(int id, double maxDomainDimension, IEnumerable<Vertex2D> vertices, bool calcPseudoNormals = false)
		{
			ID = id;
			this.maxDomainDimension = maxDomainDimension;
			this.calcPseudoNormals = calcPseudoNormals;
			this.Vertices = new List<Vertex2D>(vertices);
			this.Cells = new List<LineCell2D>();
			for (int v = 0; v < Vertices.Count - 1; ++v)
			{
				Vertex2D start = Vertices[v];
				Vertex2D end = Vertices[v + 1];
				var cell = new LineCell2D(start, end, false);
				this.Cells.Add(cell);
				start.Cells.Add(cell);
				end.Cells.Add(cell);
			}

			if (calcPseudoNormals)
			{
				//TODO: These could be useful to determine the signs of distances from vertices and edges, 
				//      but not in the way they are implemented.
				CalcPseudoNormals();
			}
		}

		public List<LineCell2D> Cells { get; }

		public CrackExtension2D CrackExtension { get; private set; }

		//HERE:
		//TODO: The front should also have an InitializeGeometry() method, instead of doing stuff in the constructor. 
		//      Then it should be injected into the crack surface class's constructor, instead of this property.
		public ICrackFront2D CrackFront { get; set; } 

		public int ID { get; }

		public List<Vertex2D> Vertices { get; }

		public void CalcLevelSets(IXModel model)
		{
			//TODO: This can be sped up significantly by only processing vertices, edges and cells that are inside a circular 
			//      or square bounding box, defined for each node.
			//TODO: This method has a lot of repetition

			this.nodalLevelSets = new Dictionary<int, double[]>();
			foreach (XNode node in model.Nodes.Values)
			{
				double phi1 = double.MaxValue; // unsigned distance to the crack surface, excluding the extension
				double phi2 = double.MaxValue; // unsigned distance to the crack front
				double phi3 = double.NaN; // signed distance to the crack surface, including the extension
				double minAbsPhi3 = double.MaxValue; // the minimum absolute phi3.

				// Process cells of the active crack. 
				foreach (LineCell2D cell in Cells)
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

				// Process vertices of the crack surface, ignoring vertices of the extension, since they are too far away. 
				foreach (Vertex2D vertex in Vertices)
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
				// No need to also process vertices of the extension, since they are too far away. 
				foreach (LineCell2D cell in CrackExtension.Cells)
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

				this.nodalLevelSets[node.ID] = new double[] { phi1, phi2, phi3 };
			}
		}

		[Conditional("DEBUG")]
		public void CheckAnglesBetweenCells()
		{
			for (int c = 0; c < Cells.Count - 1; ++c)
			{
				double[] x0 = Cells[c].Vertices[0].CoordsGlobal;
				double[] x1 = Cells[c].Vertices[1].CoordsGlobal;
				double[] x2 = Cells[c + 1].Vertices[1].CoordsGlobal;

				var v = Vector.CreateFromArray(new double[] { x1[0] - x0[0], x1[1] - x0[1] });
				var w = Vector.CreateFromArray(new double[] { x2[0] - x1[0], x2[1] - x1[1] });
				double dot = v * w;
				if (dot < 0)
				{
					throw new Exception($"The angle between cells {c} and {c + 1} is not in the [-pi/2, pi/2] range.");
				}
			}
		}

		public double[] GetLevelSetsOf(XNode node) => nodalLevelSets[node.ID];

		public void InitializeGeometry(IXModel model)
		{
			// Explicit description
			this.CrackExtension = new CrackExtension2D(this, maxDomainDimension);

			// Implicit description
			CalcLevelSets(model);
		}

		public void PropagateCrack(IXModel model, CrackFrontPropagation frontGrowth)
		{
			// Explicit description
			CrackFront.UpdateGeometry(frontGrowth);

			if (calcPseudoNormals)
			{
				CalcPseudoNormals();
			}
			this.CrackExtension = new CrackExtension2D(this, maxDomainDimension);

			// Implicit description
			//CalcLevelSets(model);
		}

		private void CalcPseudoNormals()
		{
			foreach (Vertex2D vertex in Vertices)
			{
				vertex.PseudoNormal = vertex.CalcPseudoNormal();
			}
		}
	}
}