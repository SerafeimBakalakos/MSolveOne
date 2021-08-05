using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Materials.Duplicates;

namespace MGroup.XFEM.Materials
{
    public interface IFractureMaterialField
    {
        double YoungModulus { get; }
        double EquivalentYoungModulus { get; }
        double PoissonRatio { get; }
        double EquivalentPoissonRatio { get; }

        ElasticMaterial2D FindMaterialAt(XPoint point);
    }
}
