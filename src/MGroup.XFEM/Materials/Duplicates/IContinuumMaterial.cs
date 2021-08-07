using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Constitutive;

namespace MGroup.XFEM.Materials.Duplicates
{
    public interface IContinuumMaterial : IFiniteElementMaterial
    {
        double[] Stresses { get; }
    }
}
