using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public class Vertex2D : IComparable<Vertex2D>
	{
		public Vertex2D(int id, double[] globalCoordinates, bool isExtension = false)
		{
			this.ID = id;
			this.CoordsGlobal = globalCoordinates;
			this.IsExtension = isExtension;
		}

		public int ID { get; }

		public double[] CoordsGlobal { get; }

		public List<LineCell2D> Cells { get; } = new List<LineCell2D>();

		public bool IsExtension { get; }

		public bool IsFront { get; set; }

		public double[] PseudoNormal { get; set; }

		/// <summary>
		/// Calculates the area regularized pseudonormal vector at this vertex.
		/// </summary>
		/// <returns></returns>
		public double[] CalcPseudoNormal()
		{
			var normal = Vector.CreateZero(2);
			double totalLength = 0.0;
			foreach (LineCell2D line in Cells)
			{
				normal.AxpyIntoThis(Vector.CreateFromArray(line.Normal), line.Length);
				totalLength += line.Length;
			}
			normal.ScaleIntoThis(1.0 / normal.Norm2());
			return normal.RawData;
		}

		public int CompareTo(Vertex2D other) => other.ID - this.ID;

		public override int GetHashCode() => ID.GetHashCode();

		/// <summary>
		/// The sign is +1 if <paramref name="point"/> lies in the positive half-plane of all <see cref="Cells"/> of this vertex. 
		/// The sign is -1 if <paramref name="point"/> lies in the negative half-plane of all <see cref="Cells"/> of this vertex. 
		/// The sign is 0 if <paramref name="point"/> lies in the curve of all <see cref="Cells"/> of this vertex. 
		/// </summary>
		/// <param name="point">Any 2D point.</param>
		public int SignOfDistanceOf(double[] point)
		{
			var v = Vector.CreateFromArray(CoordsGlobal);
			var p = Vector.CreateFromArray(point);
			Vector vp = p - v;

			int sign = 0;
			foreach (LineCell2D cell in Cells)
			{
				var n = Vector.CreateFromArray(cell.Normal);
				double dot = vp * n;
				if (dot > 0) //TODO: duplicate code in the same method of Edge3D
				{
					Debug.Assert(sign == 1 || sign == 0, "This point is in the positive half plane of some cells"
						+ " and in the negative of others. Invalid angles between cells");
					sign = 1;
				}
				else if (dot < 0)
				{
					Debug.Assert(sign == -1 || sign == 0, "This point is in the positive half plane of some cells"
						 + " and in the negative of others. Invalid angles between cells");
					sign = -1;
				}
				else
				{
					// If sign = 0 then the point lies on a cell's curve. The actual sign will be decided be the other cells.
					// If sign = 0 for all cells, then the final sign will also be 0 and phi3 = 0.
					sign = 0;
				}
			}
			return sign;
		}

		public double UnsignedDistanceOf(double[] point) => Utilities.Distance2D(CoordsGlobal, point);
	}
}
