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
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 3.1.3
	/// </summary>
	public class ImmersedCrackFront2D : ICrackFront2D
	{
		private readonly CrackCurve2D crackCurve;

		public ImmersedCrackFront2D(CrackCurve2D crackCurve)
		{
			this.crackCurve = crackCurve;
			Vertices = ExtractTips(crackCurve);

			// The coordinate systems are determined by the vertices, edges and cells, without enforcing any specific movement.
			CoordinateSystems = new List<CrackFrontSystem2D>();
			for (int v = 0; v < Vertices.Count; ++v)
			{
				var system = new CrackFrontSystem2D(Vertices[v]);
				CoordinateSystems.Add(system);
			}
		}

		public List<CrackFrontSystem2D> CoordinateSystems { get; }

		public List<Vertex2D> Vertices { get; }

		private static List<Vertex2D> ExtractTips(CrackCurve2D crackCurve)
		{
			// Find which vertices belong to only 1 cell. These lie on the polygon boundary.
			var vertices = new List<Vertex2D>(2);
			//foreach (Vertex2D vertex in crackCurve.Vertices)
			//{
			//	if (vertex.Cells.Count == 1)
			//	{
			//		vertices.Add(vertex);
			//	}
			//}
			vertices.Add(crackCurve.Vertices[0]);
			vertices.Add(crackCurve.Vertices[crackCurve.Vertices.Count - 1]);
			foreach (Vertex2D vertex in vertices)
			{
				vertex.IsFront = true;
			}
			return vertices;
		}

		public void UpdateGeometry(CrackFrontPropagation frontPropagation)
		{
			int numOldVertices = crackCurve.Vertices.Count; // excluding crack extension vertices, which will be removed.
			for (int v = 0; v < 2; ++v)
			{
				Vertex2D oldTip = Vertices[v];
				CrackFrontSystem2D oldSystem = CoordinateSystems[v];

				// Create new tip
				double[] newCoords = oldSystem.CalcNewTipCoords(
						oldTip.CoordsGlobal, frontPropagation.AnglesAtTips[v], frontPropagation.LengthsAtTips[v]);
				var newTip = new Vertex2D(numOldVertices + v, newCoords, false);

				// Replace it
				oldTip.IsFront = false;
				newTip.IsFront = true;
				Vertices[v] = newTip;

				// Create new cell
				LineCell2D newCell;
				if (!oldSystem.IsCounterClockwise)
				{
					newCell = new LineCell2D(newTip, oldTip, false);
				}
				else
				{
					newCell = new LineCell2D(oldTip, newTip, false);
				}

				// Complete connectivity data
				oldTip.Cells.Add(newCell);
				newTip.Cells.Add(newCell);

				// Update coordinate system (after completing connectivity data) 
				CoordinateSystems[v] = new CrackFrontSystem2D(newTip);

				// Add the new items to the crack curve
				if (!oldSystem.IsCounterClockwise)
				{
					crackCurve.Vertices.Insert(0, newTip);
					crackCurve.Cells.Insert(0, newCell);
				}
				else
				{
					crackCurve.Vertices.Add(newTip);
					crackCurve.Cells.Add(newCell);
				}
			}
		}
	}
}
