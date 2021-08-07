using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.LSM
{
    public interface IClosedGeometry : IXGeometryDescription
    {
        void UnionWith(IClosedGeometry otherGeometry);
    }
}
