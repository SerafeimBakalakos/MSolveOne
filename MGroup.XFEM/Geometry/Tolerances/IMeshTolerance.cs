using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;

namespace MGroup.XFEM.Geometry.Tolerances
{
    public interface IMeshTolerance
    {
        double CalcTolerance(IXFiniteElement element);
    }
}
