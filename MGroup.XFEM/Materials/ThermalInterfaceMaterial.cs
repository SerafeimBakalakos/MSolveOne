using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Materials
{
    public class ThermalInterfaceMaterial
    {
        public ThermalInterfaceMaterial(double interfaceConductivity)
        {
            this.InterfaceConductivity = interfaceConductivity;
        }

        public double InterfaceConductivity { get; }

        public ThermalInterfaceMaterial Clone() => new ThermalInterfaceMaterial(this.InterfaceConductivity);
    }
}
