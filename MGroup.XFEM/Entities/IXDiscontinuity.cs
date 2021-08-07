using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Enrichment;

namespace MGroup.XFEM.Entities
{
    /// <summary>
    /// Parent interface for cracks, phases or any geometric entity that interacts with the mesh and introduces enrichments.
    /// </summary>
    public interface IXDiscontinuity
    {
        int ID { get; }

        void InitializeGeometry();

        //TODO: This is not applicable for all IXDiscontinuity classes. E.g. for phase boundaries, 
        //      the interactions are determined by the phases not the actual discontinuities (phase boundaries)
        // void InteractWithMesh();

        void UpdateGeometry(IGlobalVector totalFreeDisplacements);
    }
}
