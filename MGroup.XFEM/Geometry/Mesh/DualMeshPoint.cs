using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.Mesh
{
    public class DualMeshPoint
    {
        public int[] FineElementIdx { get; set; }

        public double[] FineNaturalCoordinates { get; set; }

        public double[] FineShapeFunctions { get; set; }
    }
}
