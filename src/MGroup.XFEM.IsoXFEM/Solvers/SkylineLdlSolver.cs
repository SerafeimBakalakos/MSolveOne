using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM.Solvers
{
   public class SkylineLdlSolver : ISolver
    {
        public  Vector Solve(Matrix k, Vector f)
        {
            Matrix ksym = (k.Transpose() + k).Scale(0.5);
            SkylineMatrix ksparse = SkylineMatrix.CreateFromMatrix(ksym);
            Vector u = Vector.CreateZero(f.Length);
            ksparse.FactorLdl(false).SolveLinearSystem(f, u);
            return u;
        }
    }
}
