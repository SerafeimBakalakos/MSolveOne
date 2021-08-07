using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;

using MGroup.XFEM.Materials.Duplicates;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Materials
{
    public class MatrixInclusionsThermalMaterialField : IThermalMaterialField
    {
        private readonly ThermalMaterial matrixMaterial, inclusionMaterial;
        private readonly double matrixInclusionInterfaceConductivity, inclusionInclusionInterfaceConductivity;
        private readonly int matrixPhaseID;

        public MatrixInclusionsThermalMaterialField(ThermalMaterial matrixMaterial, ThermalMaterial inclusionMaterial, 
            double matrixInclusionInterfaceConductivity, double inclusionInclusionInterfaceConductivity, int matrixPhaseID)
        {
            this.matrixMaterial = matrixMaterial;
            this.inclusionMaterial = inclusionMaterial;
            this.matrixInclusionInterfaceConductivity = matrixInclusionInterfaceConductivity;
            this.inclusionInclusionInterfaceConductivity = inclusionInclusionInterfaceConductivity;
            this.matrixPhaseID = matrixPhaseID;
        }

        public ThermalInterfaceMaterial FindInterfaceMaterialAt(IPhaseBoundary phaseBoundary)
        {
            if ((phaseBoundary.PositivePhase.ID == matrixPhaseID) || (phaseBoundary.NegativePhase.ID == matrixPhaseID))
            {
                return new ThermalInterfaceMaterial(matrixInclusionInterfaceConductivity);
            }
            else return new ThermalInterfaceMaterial(inclusionInclusionInterfaceConductivity);
        }

        public ThermalMaterial FindMaterialAt(IPhase phase)
        {
            if (phase.ID == matrixPhaseID) return matrixMaterial.Clone();
            else return inclusionMaterial.Clone();
        }
    }
}
