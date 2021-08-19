using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Distributed.IterativeMethods.Preconditioning;

namespace MGroup.LinearAlgebra.Distributed.IterativeMethods
{
	public interface IDistributedIterativeMethod
	{
		void Clear();

		IterativeStatistics Solve(ILinearTransformation matrix, IPreconditioner preconditioner,
			IGlobalVector rhs, IGlobalVector solution, bool initialGuessIsZero);
	}
}
