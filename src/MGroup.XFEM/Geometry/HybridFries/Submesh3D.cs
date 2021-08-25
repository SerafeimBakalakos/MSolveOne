using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.HybridFries
{
    /// <summary>
    /// DTO for a subset of vertices, edges and cells.
    /// </summary>
    public class Submesh3D
    {
        public List<TriangleCell3D> Cells { get; set; }

        public List<Edge3D> Edges { get; set; }

        public List<Vertex3D> Vertices { get; set; }
    }
}
