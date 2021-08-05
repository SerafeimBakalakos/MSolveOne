using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Materials.Duplicates
{
    public class ThermalMaterial
    {
        public ThermalMaterial(double thermalConductivity, double specialHeatCoeff)
        {
            this.ThermalConductivity = thermalConductivity;
            this.SpecialHeatCoeff = specialHeatCoeff;
        }

        public double SpecialHeatCoeff { get; }

        public double ThermalConductivity { get; }

        public ThermalMaterial Clone() => new ThermalMaterial(ThermalConductivity, SpecialHeatCoeff);
    }
}
