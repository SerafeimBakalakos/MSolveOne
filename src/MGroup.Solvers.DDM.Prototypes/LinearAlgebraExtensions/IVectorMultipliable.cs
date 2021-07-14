using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions
{
    public interface IVectorMultipliable : ILinearTransformation
    {
        IVector Multiply(IVectorView lhsVector);
    }
}
