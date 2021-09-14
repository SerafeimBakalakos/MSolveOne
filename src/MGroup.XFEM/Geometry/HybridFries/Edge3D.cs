using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

//TODO: Perhaps tangent vector as property?
namespace MGroup.XFEM.Geometry.HybridFries
{
	public class Edge3D
	{
		private readonly double lengthSquared;

		public Edge3D(Vertex3D start, Vertex3D end, bool isExtension = false)
		{
			this.Start = start;
			this.End = end;
			this.IsExtension = isExtension;
			this.lengthSquared = CalcLengthSquared(start.CoordsGlobal, end.CoordsGlobal);
		}

		public List<TriangleCell3D> Cells { get; } = new List<TriangleCell3D>(2);

		public Vertex3D End { get; }

		public bool IsExtension { get; }

		public bool IsFront { get; set; }

		public double[] PseudoNormal { get; set; }

		public Vertex3D Start { get; }

		/// <summary>
		/// Calculates the area regularized pseudonormal vector at this edge.
		/// </summary>
		/// <returns></returns>
		public double[] CalcPseudoNormal()
		{
			var normal = Vector.CreateZero(3);
			double totalArea = 0.0;
			foreach (TriangleCell3D triangle in Cells)
			{
				normal.AxpyIntoThis(Vector.CreateFromArray(triangle.Normal), triangle.Area);
				totalArea += triangle.Area;
			}
			normal.ScaleIntoThis(1.0 / normal.Norm2());
			return normal.RawData;
		}

		public bool HasVertices(Vertex3D vertex0, Vertex3D vertex1)
		{
			if ((vertex0 == Start) && (vertex1 == End)) return true;
			else if ((vertex1 == Start) && (vertex0 == End)) return true;
			else return false;
		}

		/// <summary>
		/// The sign is +1 if <paramref name="point"/> lies in the positive half-space of all <see cref="Cells"/> of this edge. 
		/// The sign is -1 if <paramref name="point"/> lies in the negative half-space of all <see cref="Cells"/> of this edge. 
		/// The sign is 0 if <paramref name="point"/> lies in the plane of all <see cref="Cells"/> of this edge. 
		/// </summary>
		/// <param name="point">Any 3D point.</param>
		public int SignOfDistanceOf(double[] point)
		{
			var p = Vector.CreateFromArray(point);
			var p1 = Vector.CreateFromArray(Start.CoordsGlobal);
			Vector p1p = p - p1;

			// The next cannot be used for all points. However it is more correct for the points that are closer to this 
			// vertex than the cells/edges, since it incorporates an extra test that will fail for invalid crack geometries.
			//int sign = 0;
			//foreach (TriangleCell3D cell in Cells)
			//{
			//	var n = Vector.CreateFromArray(cell.Normal);
			//	double dot = p1p * n;
			//	if (dot > 0) //TODO: duplicate code in the same method of Vertex3D
			//	{
			//		Debug.Assert(sign == 1 || sign == 0, "This point is in the positive half space of one cell"
			//			+ " and in the negative of the other. Invalid angles between cells");
			//		sign = 1;
			//	}
			//	else if (dot < 0)
			//	{
			//		Debug.Assert(sign == -1 || sign == 0, "This point is in the positive half space of one cell"
			//			 + " and in the negative of the other. Invalid angles between cells");
			//		sign = -1;
			//	}
			//	else
			//	{
			//		// If sign = 0 then the point lies on a cell's plane. The actual sign will be decided be the othe cell.
			//		// If sign = 0 for all cells, then the final sign will also be 0 and phi3 = 0.
			//		sign = 0;
			//	}
			//}
			//return sign;

			var n = Vector.CreateFromArray(PseudoNormal);
			return Math.Sign(p1p * n);
		}

		public override string ToString() => $"(v{Start.ID} - v{End.ID})";

		// <summary>
		/// Returns <see cref="double.NaN"/> if the projection of <paramref name="point"/> onto the edge's line does not lie 
		/// between <see cref="Start"/> and <see cref="End"/>. Otherwise returns the actual distance.
		/// </summary>
		/// <param name="point">Any 3D point.</param>
		public double UnsignedDistanceOf(double[] point)
		{
			var p = Vector.CreateFromArray(point);
			var p1 = Vector.CreateFromArray(Start.CoordsGlobal);
			var p2 = Vector.CreateFromArray(End.CoordsGlobal);

			// Project the point onto the edge's line
			Vector p1p2 = p2 - p1;
			Vector p1p = p - p1;
			double m = (p1p * p1p2) / lengthSquared;
			if ((m > 0) && (m < 1))
			{
				// The projection point P0 lies between the vertices of the edge. The rejection vector is
				Vector p0p = p1p - m * p1p2;
				return p0p.Norm2();
			}
			else
			{
				return double.NaN;
			}
		}

		private static double CalcLengthSquared(double[] pointA, double[] pointB)
		{
			double dx0 = pointB[0] - pointA[0];
			double dx1 = pointB[1] - pointA[1];
			double dx2 = pointB[2] - pointA[2];
			return dx0 * dx0 + dx1 * dx1 + dx2 * dx2;
		}
	}
}
