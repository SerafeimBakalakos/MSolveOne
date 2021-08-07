using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Geometry.ConformingMesh
{
    public interface ITriangulator2D
    {
        IList<Triangle2D> CreateMesh(IEnumerable<double[]> points);
    }
}
