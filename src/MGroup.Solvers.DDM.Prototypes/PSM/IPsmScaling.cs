using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.LinearAlgebra.Matrices;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
    public interface IPsmScaling
    {
		void CalcScalingMatrices(Func<int, Matrix> getKff);

		void ScaleRhsVector(int subdomainID, Vector Fb);
    }
}
