using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Distributed.IterativeMethods.Preconditioning;

namespace MGroup.LinearAlgebra.Distributed.IterativeMethods
{
    public interface IDistributedIterativeMethod
    {
        IterativeStatistics Solve(IDistributedMatrix matrix, IPreconditioner preconditioner,
            IDistributedVector rhs, IDistributedVector solution, bool initialGuessIsZero);
    }
}
