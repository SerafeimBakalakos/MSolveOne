using MGroup.MSolve.Discretization;

namespace MGroup.MSolve.Meshes.Output.VTK
{
	/// <summary>
	/// Vertex used to represent VTK grids.
	/// Authors: Serafeim Bakalakos
	/// </summary>
	public class VtkPoint
	{
		public VtkPoint(int id, double x, double y = 0, double z = 0)
		{
			this.ID = id;
			this.X = x;
			this.Y = y;
			this.Z = z;
		}

		public VtkPoint(int id, double[] coords)
		{
			this.ID = id;
			if (coords.Length >= 1)
			{
				this.X = coords[0];
			}
			if (coords.Length >= 2)
			{
				this.Y = coords[1];
			}
			if (coords.Length >= 3)
			{
				this.Z = coords[2];
			}
		}

		public VtkPoint(int id, INode node)
		{
			this.ID = id;
			this.X = node.X;
			this.Y = node.Y;
			this.Z = node.Z;
		}

		public int ID { get; }

		public double X { get; set; }
		public double Y { get; set; }
		public double Z { get; set; }
	}
}
