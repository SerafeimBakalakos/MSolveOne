using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

//TODO: I need a fast managed LinearAlgebra for 2D and 3D geometric operations, with as little redirection(including interfaces)
//      as possible. The default LinearAlgebra library has too much redirection for these operations, especially the BLAS.
//      Also this library should operate on double[] and possibly double[,] with extension methods, instead of requiring
//      creation of Vector, Matrix objects all the time.
namespace MGroup.XFEM.Geometry.HybridFries
{
	public class TriangleCell3D
	{
		/// <summary>
		/// Create a new triangle cell of the surface mesh. 
		/// The order of vertices must be consistent in all triangles of the surface mesh.
		/// </summary>
		public TriangleCell3D(Vertex3D vertex0, Vertex3D vertex1, Vertex3D vertex2, bool isExtension = false)
			: this(new Vertex3D[] { vertex0, vertex1, vertex2 }, isExtension)
		{
		}

		/// <summary>
		/// Create a new triangle cell of the surface mesh.
		/// </summary>
		/// <param name="vertices">The order must be consistent in all triangles of the surface mesh.</param>
		public TriangleCell3D(Vertex3D[] vertices, bool isExtension = false)
		{
			if (vertices.Length != 3) throw new ArgumentException();
			this.Vertices = vertices;
			this.IsExtension = isExtension;

			var p1 = Vector.CreateFromArray(Vertices[0].CoordsGlobal);
			var p2 = Vector.CreateFromArray(Vertices[1].CoordsGlobal);
			var p3 = Vector.CreateFromArray(Vertices[2].CoordsGlobal);
			Vector p1p2 = p2 - p1;
			Vector p1p3 = p3 - p1; 
			Vector n = p1p2.CrossProduct(p1p3);
			double area2 = n.Norm2();
			this.Area = 0.5 * area2;
			n.ScaleIntoThis(1.0 / area2);
			this.Normal = n.RawData;
		}

		public double Area { get; }

		public Edge3D[] Edges { get; } = new Edge3D[3];

		public bool IsExtension { get; }

		public Vertex3D[] Vertices { get; }

		/// <summary>
		/// Unit normal vector. The normal vectors of all triangles of the surface mesh must be consistent
		/// </summary>
		public double[] Normal { get; }

		/// <summary>
		/// Return <see cref="double.NaN"/> if the projection of <paramref name="point"/> onto the triangle's plane lies 
		/// outside the triangle. 
		/// </summary>
		/// <param name="point">Any 3D point.</param>
		public double SignedDistanceOf(double[] point)
		{
			// Project the point onto the triangle's plane
			(Vector projection, double distance) = ProjectPoint(point);

			#region debug
			if (double.IsNaN(distance))
			{
				Console.WriteLine();
			}
			#endregion

			// Find the barycentric coordinates
			if (IsInsideTriangle(projection)) return distance;
			else return double.NaN;
		}

		private (Vector p0, double distance) ProjectPoint(double[] point)
		{
			var p = Vector.CreateFromArray(point);
			var p1 = Vector.CreateFromArray(Vertices[0].CoordsGlobal);
			var n = Vector.CreateFromArray(Normal);
			Vector p1p = p - p1;
			double distance = p1p * n;
			Vector p0p = distance * n;
			Vector p0 = p - p0p;
			return (p0, distance);
		}

		//TODO: This should be in a Utility class or dedicated triangle3D class.
		private bool IsInsideTriangle(Vector point)
		{
			// We will use barycentric coordinates, since they work in 3D and do not depend on tolerances. 
			// If P1,P2,P3 are the vertices of the triangle and P0 is a point (on the same plane), then the barycentric 
			// coordinate mi with respect to vertex i, is defined as: mi = |ai| / |aTot| * sign(ai * aTot),
			// where aTot = cross(PiPi+1, PiPi+2), ai = cross(P0Pi+1, P0Pi+2). 
			// P0 lies inside the triangle if 0 <= mi <= 1 for all i.
			// Optimizations for this code: |aTot| = 2 * area, sign(ai * aTot) = sign(ai * normal)
			// For more, see https://math.stackexchange.com/questions/4322/check-whether-a-point-is-within-a-3d-triangle,
			// https://en.wikipedia.org/wiki/Barycentric_coordinate_system#Determining_location_with_respect_to_a_triangle

			Vector p0 = point;
			var p1 = Vector.CreateFromArray(Vertices[0].CoordsGlobal);
			var p2 = Vector.CreateFromArray(Vertices[1].CoordsGlobal);
			var p3 = Vector.CreateFromArray(Vertices[2].CoordsGlobal);
			Vector[] vertices = { p1, p2, p3 };
			double area2 = 2 * Area;
			var n = Vector.CreateFromArray(Normal);

			for (int i = 0; i < 3; ++i)
			{
				Vector pi1 = vertices[(i + 1) % 3];
				Vector pi2 = vertices[(i + 2) % 3];
				Vector p0pi1 = pi1 - p0;
				Vector p0pi2 = pi2 - p0;
				Vector ai = p0pi1.CrossProduct(p0pi2);
				double mi = ai.Norm2() / area2 * Math.Sign(ai * n);

				if ((mi < 0) || (mi > 1))
				{
					return false;
				}
			}

			return true;
		}
	}
}
