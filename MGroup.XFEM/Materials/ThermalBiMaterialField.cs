using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;

using MGroup.XFEM.Entities;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Materials
{
    public class ThermalBiMaterialField : IThermalMaterialField
    {
        private readonly ThermalMaterial material0, material1;
        private readonly double interfaceConductivity;

        public ThermalBiMaterialField(ThermalMaterial material0, ThermalMaterial material1, double interfaceConductivity)
        {
            this.material0 = material0;
            this.material1 = material1;
            this.interfaceConductivity = interfaceConductivity;
        }

        public HashSet<int> PhasesWithMaterial0 { get; } = new HashSet<int>();
        public HashSet<int> PhasesWithMaterial1 { get; } = new HashSet<int>();

        public ThermalInterfaceMaterial FindInterfaceMaterialAt(IPhaseBoundary phaseBoundary)
        {
            Debug.Assert(
                (PhasesWithMaterial0.Contains(phaseBoundary.PositivePhase.ID)
                && PhasesWithMaterial1.Contains(phaseBoundary.NegativePhase.ID))
                || (PhasesWithMaterial0.Contains(phaseBoundary.NegativePhase.ID)
                && PhasesWithMaterial1.Contains(phaseBoundary.PositivePhase.ID)));
            return new ThermalInterfaceMaterial(interfaceConductivity);
        }

        public ThermalMaterial FindMaterialAt(IPhase phase)
        {
            if (PhasesWithMaterial0.Contains(phase.ID)) return material0.Clone();
            else
            {
                Debug.Assert(PhasesWithMaterial1.Contains(phase.ID));
                return material1.Clone();
            }
        }
    }
}
