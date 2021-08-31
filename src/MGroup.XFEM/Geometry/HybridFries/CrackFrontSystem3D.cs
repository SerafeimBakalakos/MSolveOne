using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

//TODO: Consider exposing Vector instead of double[]
namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 3.2.4
	/// </summary>
	public class CrackFrontSystem3D
	{
		public CrackFrontSystem3D(Vertex3D vertex, Vertex3D previous, Vertex3D next)
		{
			// Normal
			this.Normal = vertex.CalcPseudoNormal(); //TODO: This may also be stored in each vertex, since it is used for signed distances.

			// Tangent
			var v0 = Vector.CreateFromArray(previous.CoordsGlobal);
			var v1 = Vector.CreateFromArray(vertex.CoordsGlobal);
			var v2 = Vector.CreateFromArray(next.CoordsGlobal);
			Vector q0 = v1 - v0;
			Vector q1 = v2 - v1;
			double l0 = q0.Norm2();
			double l1 = q1.Norm2();
			Vector tangent = (1 / (l0 + l1)) * (l0 * q0 + l1 * q1);
			tangent.ScaleIntoThis(1.0 / tangent.Norm2());
			this.Tangent = tangent.RawData;

			// Extension
			Vector extension = tangent.CrossProduct(Vector.CreateFromArray(this.Normal));
			this.Extension = extension.RawData;
		}

		/// <summary>
		/// This is not necessarily a unit vector. It is orthogonal to <see cref="Normal"/> and <see cref="Tangent"/>.
		/// </summary>
		public double[] Extension { get; }

		/// <summary>
		/// Unit vector. Orthogonal to <see cref="Extension"/> and linearly independent to <see cref="Tangent"/>.
		/// </summary>
		public double[] Normal { get; }

		/// <summary>
		/// Unit vector. Orthogonal to <see cref="Extension"/> and linearly independent to <see cref="Normal"/>.
		/// </summary>
		public double[] Tangent { get; }

		/// <summary>
		/// 
		/// </summary>
		/// <param name="oldTipCoords"></param>
		/// <param name="angle">
		/// Counter-clockwise angle from the current <see cref="Extension"/> to the propagation vector.
		/// </param>
		/// <param name="length"></param>
		public double[] CalcNewTipCoords(double[] oldTipCoords, double angle, double length)
		{
			// Params angle and length are given in the coordinate system of each vertex, where the extension vector is 
			// local axis x and the normal vector is local axis y.
			var t = Vector.CreateFromArray(Extension);
			var unitT = t.Scale(1.0 / t.Norm2());
			var n = Vector.CreateFromArray(Normal); // already unit

			// Find the propagation vector in 3D.
			Vector et = (length * Math.Cos(angle)) * unitT;
			Vector en = (length * Math.Sin(angle)) * n;
			Vector propagation = et + en;

			// Find the coordinates of the new vertex
			var oldVertex = Vector.CreateFromArray(oldTipCoords);
			Vector newVertex = oldVertex + propagation;
			return newVertex.RawData;
		}

	}
}
