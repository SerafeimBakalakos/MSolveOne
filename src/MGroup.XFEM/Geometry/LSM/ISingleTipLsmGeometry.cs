using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Geometry.LSM;

//TODO: Perhaps abstract the number/type/dimension of tips and move the methods from here to IOpenLsmGeometry
namespace MGroup.XFEM.Geometry.LSM
{
    public interface ISingleTipLsmGeometry : ILsmGeometry
    {
        Dictionary<int, double> LevelSetsTip { get; }
    }
}
