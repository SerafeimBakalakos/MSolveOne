using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM.Solvers
{
   public class DenseCholeskySolver : ISolver
    {
        public  Vector Solve(Matrix k, Vector f)
        {
            Matrix ksym = (k.Transpose() + k).Scale(0.5);
            Vector u = Vector.CreateZero(f.Length);
            ksym.FactorCholesky().SolveLinearSystem(f, u);
            return u;
        }

    }
}
