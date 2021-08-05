using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Materials
{
    public interface IStructuralMaterialField
    {
        CohesiveInterfaceMaterial FindInterfaceMaterialAt(IPhaseBoundary phaseBoundary);

        IContinuumMaterial FindMaterialAt(IPhase phase);
    }
}
