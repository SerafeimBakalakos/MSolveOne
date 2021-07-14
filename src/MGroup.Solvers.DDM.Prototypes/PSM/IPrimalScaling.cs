using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
    public interface IPrimalScaling
    {
        Dictionary<int, SparseVector> DistributeNodalLoads(Table<INode, IDofType, double> nodalLoads);
    }
}
