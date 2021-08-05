using System;

namespace MGroup.XFEM.Output.Vtk
{
    /// <summary>
    /// Vertex used to represent VTK grids.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    public class VtkPoint
    {
        public VtkPoint(int id, double[] coordinates)
        {
            this.ID = id;
            this.Coordinates = coordinates;
        }

        public int ID { get; }

        public double[] Coordinates { get; }

        public double[] Get3DCoordinates()
        {
            if (Coordinates.Length == 1) return new double[] { Coordinates[0], 0, 0 };
            else if (Coordinates.Length == 2) return new double[] { Coordinates[0], Coordinates[1], 0 };
            else if (Coordinates.Length == 3) return Coordinates;
            else throw new Exception();
        }
    }
}
