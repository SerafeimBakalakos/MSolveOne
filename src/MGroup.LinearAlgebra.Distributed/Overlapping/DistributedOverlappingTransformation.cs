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
	public class DistributedOverlappingTransformation : ILinearTransformation
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
		public delegate void MultiplyLocalVector(int computeNodeID, Vector input, Vector output);

		private readonly IComputeEnvironment environment;
		private readonly Func<IGlobalVector, DistributedOverlappingVector> checkCompatibleVector;
		private readonly DistributedOverlappingIndexer indexer;
		private readonly MultiplyLocalVector multiplyMatrixVectorPerComputeNode;

		public DistributedOverlappingTransformation(IComputeEnvironment environment, DistributedOverlappingIndexer indexer,
			Func<IGlobalVector, DistributedOverlappingVector> checkCompatibleVector, 
			MultiplyLocalVector multiplyMatrixVectorPerComputeNode)
		{
			this.environment = environment;
			this.indexer = indexer;
			this.checkCompatibleVector = checkCompatibleVector;
			this.multiplyMatrixVectorPerComputeNode = multiplyMatrixVectorPerComputeNode;
		}

		public IDistributedIndexer Indexer => indexer;


		public void MultiplyVector(IGlobalVector input, IGlobalVector output)
		{
			DistributedOverlappingVector inputCasted = checkCompatibleVector(input);
			DistributedOverlappingVector outputCasted = checkCompatibleVector(output);
			MultiplyVector(inputCasted, outputCasted);
		}

		public void MultiplyVector(DistributedOverlappingVector input, DistributedOverlappingVector output)
		{
			checkCompatibleVector(input);
			checkCompatibleVector(output);

			Action<int> multiplyLocal = nodeID =>
			{
				Vector localX = input.LocalVectors[nodeID];
				Vector localY = output.LocalVectors[nodeID];
				multiplyMatrixVectorPerComputeNode(nodeID, localX, localY);
			};
			environment.DoPerNode(multiplyLocal);

			output.SumOverlappingEntries();
		}
	}
}
