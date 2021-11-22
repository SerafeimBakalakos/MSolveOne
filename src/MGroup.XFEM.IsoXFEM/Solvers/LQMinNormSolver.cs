using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Orthogonalization;
using MGroup.LinearAlgebra.Vectors;
using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM.Solvers
{
    public class LQMinNormSolver : ISolver
    {
        public Vector Solve(Matrix k, Vector f)
        {
            Matrix kSymmetric = (k.Transpose() + k).Scale(0.5);
            LQFactorization kLQ = kSymmetric.FactorLQ();
            Vector u = kLQ.SolveMinNorm(f);
            return u;
        }
    }
}
