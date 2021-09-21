using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

//TODO: Consider exposing Vector instead of double[]
namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// Counter-clockwise coordinate system centered around a crack tip.
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

			// Figure out the direction of the tangent vector
			Vertex2D start, end;
			if (tip == segment.Vertices[0])
			{
				start = segment.Vertices[1];
				end = segment.Vertices[0];
				IsCenteredAroundStartTip = true;
				this.Normal = new double[] { -segment.Normal[0], -segment.Normal[1] };
			}
			else
			{
				start = segment.Vertices[0];
				end = segment.Vertices[1];
				IsCenteredAroundStartTip = false;
				this.Normal = new double[] { +segment.Normal[0], +segment.Normal[1] };
			}

			// Calculate the tangent vector
			var v0 = Vector.CreateFromArray(start.CoordsGlobal);
			var v1 = Vector.CreateFromArray(end.CoordsGlobal);
			Vector extension = v1 - v0;
			extension.ScaleIntoThis(1.0 / segment.Length);
			this.Extension = extension.RawData;
		}

		public bool IsCenteredAroundStartTip { get; }

		/// <summary>
		/// See <see cref="ICrackTipSystem.Normal;"/>.
		/// </summary>
		/// <remarks>
		/// Starting from the direction of <see cref="Extension"/>, the normal vector will be at a PI/2 counter-clockwise angle.
		/// Thus, at the end tip, it will point towards the region of positive signed distances (from the crack body).
		/// In constrast, at the start tip, it will point towards the region of negative signed distances. 
		/// </remarks>
		public double[] Normal { get; }

		/// <summary>
		/// See <see cref="ICrackTipSystem.Extension"/>.
		/// </summary>
		/// <remarks>
		/// This will point away from the crack body for both crack tips, at start and at end.
		/// </remarks>
		public double[] Extension { get; }

		public double[] TipCoordsGlobal => tip.CoordsGlobal;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="angle">Counter-clockwise angle from the current <see cref="Extension"/> to the propagation vector.</param>
		/// <param name="length"></param>
		public double[] ExtendTowards(double angle, double length)
		{
			// p1 = l * cosa * t
			// p2 = l * sina * n
			// p = p1 + p2
			// x1 = x0 + p
			double cosa = length * Math.Cos(angle);
			double sina = length * Math.Sin(angle);
			var result = new double[2];
			result[0] = tip.CoordsGlobal[0] + cosa * Extension[0] + sina * Normal[0];
			result[1] = tip.CoordsGlobal[1] + cosa * Extension[1] + sina * Normal[1];
			return result;
		}

		public double[] RotateGlobalStressTensor(double[] globalStresses)
		{
			Debug.Assert(globalStresses.Length == 3);
			double t11 = globalStresses[0];
			double t22 = globalStresses[1];
			double t12 = globalStresses[2];

			double cosa = Extension[0];
			double sina = Extension[1];
			double cos2a = 2 * cosa * cosa - 1;
			double sin2a = 2 * cosa * sina;

			var result = new double[3];
			result[0] = 0.5 * (t11 + t22) + 0.5 * (t11 - t22) * cos2a + t12 * sin2a;
			result[1] = 0.5 * (t11 + t22) + 0.5 * (t11 - t22) * cos2a - t12 * sin2a;
			result[2] = t12 * cos2a - 0.5 * (t11 - t22) * sin2a;

			return result;
		}
	}
}
