using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM.Solvers
{
    public interface ISolver
    {
        Vector Solve(Matrix k, Vector f);
    }
}
