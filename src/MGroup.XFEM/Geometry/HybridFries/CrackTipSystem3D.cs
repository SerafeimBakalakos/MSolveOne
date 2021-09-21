using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Geometry.Boundaries;

//TODO: Consider exposing Vector instead of double[]
namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 3.2.4
	/// </summary>
	public class CrackTipSystem3D : ICrackTipSystem
	{
		private readonly Vertex3D tip;

		public CrackTipSystem3D(Vertex3D vertex, Vertex3D previous, Vertex3D next, IDomainBoundary3D boundary)
		{
			this.tip = vertex;
			if (vertex.Position == VertexPosition.TipActive)
			{
				if ((previous.Position == VertexPosition.TipActive) && (next.Position == VertexPosition.TipActive))
				{
					// Active tip vertex that does not lie on the domain's boundary and thus is unconstrained.
					(Normal, Tangent, Extension) = CalcRegularVectors(vertex, previous, next);
				}
				else
				{
					Vertex3D otherBoundaryTip;
					if (previous.Position == VertexPosition.TipInactive)
					{
						otherBoundaryTip = previous;
					}
					else if (next.Position == VertexPosition.TipInactive)
					{
						otherBoundaryTip = next;
					}
					else
					{
						throw new ArgumentException(
							"The vertex is an active tip surrounded by 2 inactive tips. This should not have happened.");
					}
					(Normal, Tangent, Extension) = CalcConstrainedVectors(vertex, otherBoundaryTip, boundary);
				}
			}
			else if (vertex.Position == VertexPosition.TipInactive)
			{
				// These are needed to construct the crack extension.
				(Normal, Tangent, Extension) = CalcRegularVectors(vertex, previous, next);
			}
			else
			{
				throw new ArgumentException("The vertex provided is not a tip.");
			}
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

		public double[] TipCoordsGlobal => tip.CoordsGlobal;


		/// <summary>
		/// 
		/// </summary>
		/// <param name="oldTipCoords"></param>
		/// <param name="angle">
		/// Counter-clockwise angle from the current <see cref="Extension"/> to the propagation vector.
		/// </param>
		/// <param name="length"></param>
		public double[] ExtendTowards(double angle, double length)
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
			var tipCoords = Vector.CreateFromArray(tip.CoordsGlobal);
			Vector result = tipCoords + propagation;
			return result.RawData;
		}

		private static (double[] normal, double[] tangent, double[] extension) CalcConstrainedVectors(Vertex3D tip, 
			Vertex3D otherBoundaryTip, IDomainBoundary3D boundary)
		{
			// The extension vector is defined from the neighboring boundary tip to the boundary tip of interest. 
			Vector extension = Vector.CreateFromArray(tip.CoordsGlobal) - Vector.CreateFromArray(otherBoundaryTip.CoordsGlobal);
			extension.ScaleIntoThis(1.0 / extension.Norm2());

			// The tangent vector must be orthogonal to the domain boundary. Its orientation will be determined later.
			var tangent = Vector.CreateFromArray(boundary.CalcNormalAtBoundaryPoint(tip.CoordsGlobal));

			// The normal vector is orthogonal to the extension and tangent. Its orientation will be determined later.
			Vector normal = extension.CrossProduct(tangent);
			normal.ScaleIntoThis(1.0 / normal.Norm2());

			// The direction of the normal vector must be congruent with the normal vector defined at the tip through regular 
			// means.
			double[] normalAtVertex = tip.PseudoNormal;
			double dot = normal * Vector.CreateFromArray(normalAtVertex);
			if (dot == 0)
			{
				throw new NotImplementedException();
			}
			else if (dot < 0)
			{
				normal.ScaleIntoThis(-1.0);
				tangent.ScaleIntoThis(-1.0);
			}

			// Also replace the normal vector at the tip
			tip.PseudoNormal = normal.RawData;

			// Tangent
			return (normal.RawData, tangent.RawData, extension.RawData);
		}

		/// <summary>
		/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
		/// section 3.2.4
		/// </summary>
		private static (double[] normal, double[] tangent, double[] extension) CalcRegularVectors(Vertex3D vertex, Vertex3D previous, Vertex3D next)
		{
			// Normal
			double[] normal = vertex.PseudoNormal;

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

			// Extension
			Vector extension = tangent.CrossProduct(Vector.CreateFromArray(normal));

			return (normal, tangent.RawData, extension.RawData);
		}
	}
}
