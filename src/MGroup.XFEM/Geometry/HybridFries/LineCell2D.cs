using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public class LineCell2D
	{
		private readonly double lengthSquared;

		/// <summary>
		/// Create a new line segment cell of the curve mesh.
		/// </summary>
		/// <param name="vertices">The order must be consistent in all line segments of the curve mesh.</param>
		public LineCell2D(Vertex2D start, Vertex2D end, bool isExtension = false) 
			: this(new Vertex2D[] { start, end }, isExtension)
		{
		}


		/// <summary>
		/// Create a new line segment cell of the curve mesh.
		/// </summary>
		/// <param name="vertices">The order must be consistent in all line segments of the curve mesh.</param>
		public LineCell2D(Vertex2D[] vertices, bool isExtension = false)
		{
			if (vertices.Length != 2) throw new ArgumentException();
			this.Vertices = vertices;
			this.IsExtension = isExtension;

			double dx = vertices[1].CoordsGlobal[0] - vertices[0].CoordsGlobal[0];
			double dy = vertices[1].CoordsGlobal[1] - vertices[0].CoordsGlobal[1];
			this.lengthSquared = dx * dx + dy * dy;
			this.Length = Math.Sqrt(lengthSquared);
			double cos = dx / Length;
			double sin = dy / Length;
			this.Normal = new double[] { -sin, cos };
		}

		public bool IsExtension { get; }

		public double Length { get; }

		/// <summary>
		/// Unit normal vector. The normal vectors of all line segments of the curve mesh must be consistent
		/// </summary>
		public double[] Normal { get; }

		public Vertex2D[] Vertices { get; }

		/// <summary>
		/// Return <see cref="double.NaN"/> if the projection of <paramref name="point"/> onto the segment's line lies 
		/// outside the segment. 
		/// </summary>
		/// <param name="point">Any 2D point.</param>
		public double SignedDistanceOf(double[] point)
		{
			var p = Vector.CreateFromArray(point);
			var p1 = Vector.CreateFromArray(Vertices[0].CoordsGlobal);
			var p2 = Vector.CreateFromArray(Vertices[1].CoordsGlobal);

			// Project the point onto the segment's line
			Vector p1p2 = p2 - p1;
			Vector p1p = p - p1;
			double m = (p1p * p1p2) / lengthSquared;
			if ((m > 0) && (m < 1))
			{
				// The projection point P0 lies between the vertices of the segment. The rejection vector is
				Vector p0p = p1p - m * p1p2;

				// The sign can be found by projecting onto the normal vector of the segment
				double sign = Math.Sign(p0p * Vector.CreateFromArray(Normal));

				// The signed distance is
				return sign * p0p.Norm2();
			}
			else
			{
				return double.NaN;
			}
		}
	}
}
