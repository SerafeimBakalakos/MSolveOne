using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Exceptions;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Environments;

//TODO: Actually if IDistributedVector exposes its local subvectors, it probably does not matter if the lhs vector is 
//      DistributedOverlappingVector.
namespace MGroup.LinearAlgebra.Distributed.Overlapping
{
    public class DistributedOverlappingMatrix : IDistributedMatrix
    {
        /// <summary>
        /// Multiplies a matrix (A) with the input vector (x) and writes the result in the output vector (y), such that 
        /// y = A * x. An equivalent operator y = opA(x) can also be used, instead of an explict matrix A. In any case, this 
        /// operation is done for a specific <see cref="MGroup.Environments.ComputeNode"/>.
        /// </summary>
        /// <param name="computeNodeID">
        /// The ID of the <see cref="MGroup.Environments.ComputeNode"/> for which the matrix-vector multiplication will be 
        /// performed. The matrix (A) or operator (opA), and the 2 vectors must all belong to this 
        /// <see cref="MGroup.Environments.ComputeNode"/>, wlthough this will not be checked explicitly.
        /// </param>
        /// <param name="input">
        /// The left-hand-side vector multiplied by the matrix A or used as input for operator opA. Its dimensions must match 
        /// the number of columns of A or the dimension of the input space of opA respectively.</param>
        /// <param name="output">
        /// The right-hand-side vector where the result of the multiplication will be written to. Its dimensions must match 
        /// the number of rows of A or the dimension of the output space of opA respectively. On input it is not guaranteed to
        /// be a zero vector, thus this method must make sure to zero it out, if needed.
        /// </param>
        public delegate void MultiplyMatrixVector(int computeNodeID, Vector input, Vector output);

        private readonly IComputeEnvironment environment;
        private readonly DistributedOverlappingIndexer indexer;
        private readonly MultiplyMatrixVector multiplyMatrixVectorPerComputeNode;

        public DistributedOverlappingMatrix(IComputeEnvironment environment, DistributedOverlappingIndexer indexer,
            MultiplyMatrixVector multiplyMatrixVectorPerComputeNode)
        {
            this.environment = environment;
            this.indexer = indexer;
            this.multiplyMatrixVectorPerComputeNode = multiplyMatrixVectorPerComputeNode;
        }

        public IDistributedIndexer Indexer => indexer;


        public void Multiply(IGlobalVector lhs, IGlobalVector rhs)
        {
            if ((lhs is DistributedOverlappingVector lhsCasted) && (rhs is DistributedOverlappingVector rhsCasted))
            {
                Multiply(lhsCasted, rhsCasted);
            }
            else
            {
                throw new ArgumentException(
                    "This operation is legal only if the left-hand-side and righ-hand-side vectors are distributed" +
                    " with overlapping entries.");
            }
        }

        public void Multiply(DistributedOverlappingVector lhs, DistributedOverlappingVector rhs)
        {
            //TODOMPI: also check that environment is the same between A,x and A,y
            Debug.Assert(this.indexer.Matches(lhs.Indexer) /*&& (this.environment == lhs.environment)*/);
            Debug.Assert(this.indexer.Matches(rhs.Indexer) /*&& (this.environment == rhs.environment)*/);

            Action<int> multiplyLocal = nodeID =>
            {
                Vector localX = lhs.LocalVectors[nodeID];
                Vector localY = rhs.LocalVectors[nodeID];
                multiplyMatrixVectorPerComputeNode(nodeID, localX, localY);
            };
            environment.DoPerNode(multiplyLocal);

            rhs.SumOverlappingEntries();
        }
    }
}
