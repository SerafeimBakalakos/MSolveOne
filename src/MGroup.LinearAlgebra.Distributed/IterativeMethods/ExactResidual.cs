using MGroup.LinearAlgebra.Vectors;

namespace MGroup.LinearAlgebra.Distributed.IterativeMethods
{
    internal static class ExactResidual
    {
        internal static IDistributedVector Calculate(IDistributedMatrix matrix,
            IDistributedVector rhs, IDistributedVector solution)
        {
            IDistributedVector residual = rhs.CreateZeroVectorWithSameFormat();
            Calculate(matrix, rhs, solution, residual);
            return residual;
        }

        internal static void Calculate(IDistributedMatrix matrix, IDistributedVector rhs,
            IDistributedVector solution, IDistributedVector residual)
        {
            //TODO: There is a BLAS operation y = y + a * A*x, that would be perfect for here. rhs.Copy() and then that.
            matrix.Multiply(solution, residual);
            residual.LinearCombinationIntoThis(-1.0, rhs, 1.0);
        }

    }
}