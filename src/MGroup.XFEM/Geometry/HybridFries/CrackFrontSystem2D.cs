using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

//TODO: Consider exposing Vector instead of double[]
namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 3.1.4
	/// </summary>
	public class CrackFrontSystem2D
	{
		public CrackFrontSystem2D(Vertex2D tip)
		{
			if (tip.Cells.Count != 1)
			{
				throw new ArgumentException($"Vertex {tip.ID} is not a tip.");
			}

			// The normal vector is the same as the line segment
			LineCell2D segment = tip.Cells[0];
			this.Normal = segment.Normal;

			// Figure out the direction of the tangent vector
			Vertex2D start, end;
			if (tip == segment.Vertices[1])
			{
				start = segment.Vertices[0];
				end = segment.Vertices[1];
				IsCounterClockwise = true;
			}
			else
			{
				start = segment.Vertices[1];
				end = segment.Vertices[0];
				IsCounterClockwise = false;
			}

			// Calculate the tangent vector
			var v0 = Vector.CreateFromArray(start.CoordsGlobal);
			var v1 = Vector.CreateFromArray(end.CoordsGlobal);
			Vector tangent = v1 - v0;
			tangent.ScaleIntoThis(1.0 / segment.Length);
			this.Tangent = tangent.RawData;
		}

		public bool IsCounterClockwise { get; }

		/// <summary>
		/// Unit vector. Orthogonal to <see cref="Tangent"/>.
		/// </summary>
		public double[] Normal { get; }

		/// <summary>
		/// Unit vector. Orthogonal to <see cref="Normal"/>.
		/// </summary>
		public double[] Tangent { get; }

		public double[] CalcNewTipCoords(double[] oldTipCoords, double angle, double length)
		{
			double cT = length * Math.Cos(angle);
			double cN = length * Math.Sin(angle);
			var newTipCoords = new double[2];
			if (IsCounterClockwise)
			{
				// p1 = l * cosa * t
				// p2 = l * sina * n
				// p = p1 + p2
				// x1 = x0 + p
				newTipCoords[0] = oldTipCoords[0] + cT * Tangent[0] + cN * Normal[0];
				newTipCoords[1] = oldTipCoords[1] + cT * Tangent[1] + cN * Normal[1];
			}
			else
			{
				// p1 = l * cosa * t
				// p2 = - l * sina * n
				// p = p1 + p2
				// x1 = x0 + p
				newTipCoords[0] = oldTipCoords[0] + cT * Tangent[0] - cN * Normal[0];
				newTipCoords[1] = oldTipCoords[1] + cT * Tangent[1] - cN * Normal[1];
			}
			return newTipCoords;
		}
	}
}
