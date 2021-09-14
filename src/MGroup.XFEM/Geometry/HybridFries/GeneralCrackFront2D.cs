using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Extensions;
using MGroup.XFEM.Geometry.Boundaries;

namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 3.1.3
	/// </summary>
	public class GeneralCrackFront2D : ICrackFront2D
	{
		private readonly CrackCurve2D crackCurve;
		private readonly IDomainBoundary2D domainBoundary;

		public GeneralCrackFront2D(CrackCurve2D crackCurve, IDomainBoundary2D domainBoundary)
		{
			this.crackCurve = crackCurve;
			this.domainBoundary = domainBoundary;

			Vertices = new List<Vertex2D>(2);
			Vertices.Add(crackCurve.Vertices[0]);
			Vertices.Add(crackCurve.Vertices[crackCurve.Vertices.Count - 1]);

			ActiveTips = new List<int>(2);
			DetermineActiveTips();

			// The coordinate systems are determined by the vertices, edges and cells, without enforcing any specific movement.
			CoordinateSystems = new List<CrackFrontSystem2D>();
			for (int v = 0; v < Vertices.Count; ++v)
			{
				var system = new CrackFrontSystem2D(Vertices[v]);
				CoordinateSystems.Add(system);
			}
		}

		public List<int> ActiveTips { get; }

		public List<CrackFrontSystem2D> CoordinateSystems { get; }

		public List<Vertex2D> Vertices { get; }

		public void UpdateGeometry(CrackFrontPropagation frontPropagation)
		{
			Debug.Assert(frontPropagation.AnglesAtTips.Length == ActiveTips.Count);

			int numOldVertices = crackCurve.Vertices.Count; // excluding crack extension vertices, which will be removed.
			for (int i = 0; i < ActiveTips.Count; ++i)
			{
				int vertexIdx = ActiveTips[i];
				Vertex2D oldTip = Vertices[vertexIdx];
				CrackFrontSystem2D oldSystem = CoordinateSystems[vertexIdx];

				// Create new tip
				double[] newCoords = oldSystem.CalcNewTipCoords(
						oldTip.CoordsGlobal, frontPropagation.AnglesAtTips[i], frontPropagation.LengthsAtTips[i]);
				var newTip = new Vertex2D(numOldVertices + i, newCoords, false);

				// Replace it
				oldTip.Position = VertexPosition.Internal;
				Vertices[vertexIdx] = newTip;

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
				CoordinateSystems[vertexIdx] = new CrackFrontSystem2D(newTip);

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

			DetermineActiveTips();
		}

		private void DetermineActiveTips()
		{
			ActiveTips.Clear();
			for (int v = 0; v < Vertices.Count; ++v)
			{
				Vertex2D tip = Vertices[v];
				if (domainBoundary.SurroundsPoint(tip.CoordsGlobal))
				{
					ActiveTips.Add(v);
					tip.Position = VertexPosition.TipActive;
				}
				else
				{
					tip.Position = VertexPosition.TipInactive;
				}
			}

			if (ActiveTips.Count < 1)
			{
				throw new Exception("All tips lie outside the domain.");
			}
		}
	}
}
