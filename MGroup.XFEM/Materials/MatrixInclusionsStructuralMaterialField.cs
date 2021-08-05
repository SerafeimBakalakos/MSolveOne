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
    public class MatrixInclusionsStructuralMaterialField : IStructuralMaterialField
    {
        private readonly IContinuumMaterial matrixMaterial, inclusionMaterial;
        private readonly CohesiveInterfaceMaterial interfaceMaterial;
        private readonly int matrixPhaseID;

        public MatrixInclusionsStructuralMaterialField(IContinuumMaterial matrixMaterial, IContinuumMaterial inclusionMaterial,
            CohesiveInterfaceMaterial interfaceMaterial, int matrixPhaseID)
        {
            this.matrixMaterial = matrixMaterial;
            this.inclusionMaterial = inclusionMaterial;
            this.interfaceMaterial = interfaceMaterial;
            this.matrixPhaseID = matrixPhaseID;
        }

        public CohesiveInterfaceMaterial FindInterfaceMaterialAt(IPhaseBoundary phaseBoundary)
        {
            return interfaceMaterial.Clone();
        }

        public IContinuumMaterial FindMaterialAt(IPhase phase)
        {
            if (phase.ID == matrixPhaseID) return (IContinuumMaterial)matrixMaterial.Clone();
            else return (IContinuumMaterial)inclusionMaterial.Clone();
        }
    }
}
