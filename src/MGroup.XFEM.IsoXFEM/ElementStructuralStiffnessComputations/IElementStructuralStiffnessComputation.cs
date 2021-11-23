using MGroup.LinearAlgebra.Matrices;
using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM.ElementStructuralStiffnessComputations
{
    public interface IElementStructuralStiffnessComputation
    {
       Matrix ElementStructuralStiffnessComputation(Matrix coordinatesOfElement, Matrix elasticityMatrix, double thickness);
    }
}