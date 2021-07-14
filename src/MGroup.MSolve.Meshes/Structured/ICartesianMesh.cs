using System;
using System.Collections.Generic;
using System.Text;

//TODO: Perhaps a similar interface for triangular structured meshes. The generalization of triangle is called simplex so perhaps 
//      ISimplicialStructuredMesh. There is actually a lot of math behind these: https://en.wikipedia.org/wiki/Simplex.
namespace MGroup.MSolve.Meshes.Structured
{
    /// <summary>
    /// Special case of structured mesh, where elements are quadrilaterals in 2D or hexahedrals in 3D.
    /// </summary>
    public interface ICartesianMesh : IStructuredMesh
    {
        int[] NumElements { get; }
    }
}
