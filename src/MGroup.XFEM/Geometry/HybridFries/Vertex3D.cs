using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public class Vertex3D : IComparable<Vertex3D>
	{
		public Vertex3D(int id, double[] globalCoordinates, bool isExtension = false)
		{
			this.ID = id;
			this.CoordsGlobal = globalCoordinates;
			this.Position = isExtension ? VertexPosition.Extension : VertexPosition.Internal;
		}

		public int ID { get; }

		public double[] CoordsGlobal { get; }

		public List<TriangleCell3D> Cells { get; } = new List<TriangleCell3D>();

		public List<Edge3D> Edges { get; } = new List<Edge3D>();

		public double[] PseudoNormal { get; set; }

		public VertexPosition Position { get; set; }

		/// <summary>
		/// Calculates the area regularized pseudonormal vector at this vertex.
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

		public int CompareTo(Vertex3D other) => other.ID - this.ID;

		public override int GetHashCode() => ID.GetHashCode();

		/// <summary>
		/// The sign is +1 if <paramref name="point"/> lies in the positive half-space of all <see cref="Cells"/> of this vertex. 
		/// The sign is -1 if <paramref name="point"/> lies in the negative half-space of all <see cref="Cells"/> of this vertex. 
		/// The sign is 0 if <paramref name="point"/> lies in the plane of all <see cref="Cells"/> of this vertex. 
		/// </summary>
		/// <param name="point">Any 3D point.</param>
		public int SignOfDistanceOf(double[] point)
		{
			var v = Vector.CreateFromArray(CoordsGlobal);
			var p = Vector.CreateFromArray(point);
			Vector vp = p - v;

			// The next cannot be used for all points. However it is more correct for the points that are closer to this 
			// vertex than the cells/edges, since it incorporates an extra test that will fail for invalid crack geometries.
			//int sign = 0;
			//foreach (TriangleCell3D cell in Cells)
			//{
			//	var n = Vector.CreateFromArray(cell.Normal);
			//	double dot = vp * n;
			//	if (dot > 0) //TODO: duplicate code in the same method of Edge3D
			//	{
			//		Debug.Assert(sign == 1 || sign == 0, "This point is in the positive half space of some cells"
			//			+ " and in the negative of others. Invalid angles between cells");
			//		sign = 1;
			//	}
			//	else if (dot < 0)
			//	{
			//		Debug.Assert(sign == -1 || sign == 0, "This point is in the positive half space of some cells"
			//			 + " and in the negative of others. Invalid angles between cells");
			//		sign = -1;
			//	}
			//	else
			//	{
			//		// If sign = 0 then the point lies on a cell's plane. The actual sign will be decided be the other cells.
			//		// If sign = 0 for all cells, then the final sign will also be 0 and phi3 = 0.
			//		sign = 0;
			//	}
			//}
			//return sign;

			var n = Vector.CreateFromArray(PseudoNormal);
			return Math.Sign(vp * n);
		}

		public double UnsignedDistanceOf(double[] point) => Utilities.Distance3D(CoordsGlobal, point);
	}
}
