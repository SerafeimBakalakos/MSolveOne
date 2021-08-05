using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Geometry;

namespace MGroup.XFEM.Geometry.LSM
{
    public interface ILsmGeometry : IXGeometryDescription
    {
        Dictionary<int, double> LevelSets { get; }
    }
}
