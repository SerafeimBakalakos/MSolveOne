using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.ElementGeometry;

namespace MGroup.XFEM.Geometry.Mesh
{
    public interface IDualMesh
    {
        int Dimension { get; }

        IStructuredMesh CoarseMesh { get; }

        IStructuredMesh FineMesh { get; }

        /// <summary>
        /// If the node in the fine mesh does not correspond to a node in the coarse mesh, -1 will be returned
        /// </summary>
        /// <param name="fineNodeID"></param>
        int MapNodeFineToCoarse(int fineNodeID);

        int MapNodeIDCoarseToFine(int coarseNodeID);

        int MapElementFineToCoarse(int fineElementID);

        int[] MapElementCoarseToFine(int coarseElementID);

        double[] MapPointFineNaturalToCoarseNatural(int[] fineElementIdx, double[] coordsFineNatural);

        DualMeshPoint CalcShapeFunctions(int coarseElementID, double[] coarseNaturalCoords);
    }
}
