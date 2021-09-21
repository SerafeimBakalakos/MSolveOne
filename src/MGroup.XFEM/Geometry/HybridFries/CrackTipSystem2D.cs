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
	public class CrackTipSystem2D : ICrackTipSystem
	{
		private readonly Vertex2D tip;

		public CrackTipSystem2D(Vertex2D tip)
		{
			if (tip.Cells.Count != 1)
			{
				throw new ArgumentException($"Vertex {tip.ID} is not a tip.");
			}
			this.tip = tip;

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
			Vector extension = v1 - v0;
			extension.ScaleIntoThis(1.0 / segment.Length);
			this.Extension = extension.RawData;
		}

		public bool IsCounterClockwise { get; }

		public double[] Normal { get; }

		public double[] Extension { get; }

		public double[] TipCoordsGlobal => tip.CoordsGlobal;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="angle">Counter-clockwise angle from the current <see cref="Extension"/> to the propagation vector.</param>
		/// <param name="length"></param>
		public double[] ExtendTowards(double angle, double length)
		{
			double cT = length * Math.Cos(angle);
			double cN = length * Math.Sin(angle);
			var result = new double[2];
			if (IsCounterClockwise)
			{
				// p1 = l * cosa * t
				// p2 = l * sina * n
				// p = p1 + p2
				// x1 = x0 + p
				result[0] = tip.CoordsGlobal[0] + cT * Extension[0] + cN * Normal[0];
				result[1] = tip.CoordsGlobal[1] + cT * Extension[1] + cN * Normal[1];
			}
			else
			{
				// p1 = l * cosa * t
				// p2 = - l * sina * n
				// p = p1 + p2
				// x1 = x0 + p
				result[0] = tip.CoordsGlobal[0] + cT * Extension[0] - cN * Normal[0];
				result[1] = tip.CoordsGlobal[1] + cT * Extension[1] - cN * Normal[1];
			}
			return result;
		}
	}
}
