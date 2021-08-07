using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Materials
{
    public interface IThermalMaterialField
    {
        ThermalInterfaceMaterial FindInterfaceMaterialAt(IPhaseBoundary phaseBoundary);

        ThermalMaterial FindMaterialAt(IPhase phase);
    }
}
