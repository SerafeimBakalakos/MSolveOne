using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DDM.LinearSystem
{
	public class DistributedOverlappingMatrix<TMatrix> : IGlobalMatrix
		where TMatrix : IMatrix
	{
		private readonly DistributedOverlappingIndexer indexer;
		private readonly Func<IGlobalVector, DistributedOverlappingVector> checkCompatibleVector;
		private readonly Func<IGlobalMatrix, DistributedOverlappingMatrix<TMatrix>> checkCompatibleMatrix;

		public DistributedOverlappingMatrix(IComputeEnvironment environment, DistributedOverlappingIndexer indexer, 
			Guid format, Func<IGlobalVector, DistributedOverlappingVector> checkCompatibleVector,
			Func<IGlobalMatrix, DistributedOverlappingMatrix<TMatrix>> checkCompatibleMatrix)
		{
			Environment = environment;
			this.indexer = indexer;
			Format = format;
			this.checkCompatibleVector = checkCompatibleVector;
			this.checkCompatibleMatrix = checkCompatibleMatrix;
		}

		public IComputeEnvironment Environment { get; }

		public Guid Format { get; }

		public IDistributedIndexer Indexer => indexer;

		public ConcurrentDictionary<int, TMatrix> LocalMatrices { get; } = new ConcurrentDictionary<int, TMatrix>();

		public void AxpyIntoThis(IGlobalMatrix otherMatrix, double otherCoefficient)
		{
			DistributedOverlappingMatrix<TMatrix> distributedOther = checkCompatibleMatrix(otherMatrix);
			Action<int> localOperation = nodeID =>
			{
				TMatrix thisSubdomainMatrix = this.LocalMatrices[nodeID];
				TMatrix otherSubdomainMatrix = distributedOther.LocalMatrices[nodeID];
				thisSubdomainMatrix.AxpyIntoThis(otherSubdomainMatrix, otherCoefficient);
			};
			Environment.DoPerNode(localOperation);
		}

		public void Clear()
		{
			Environment.DoPerNode(nodeID => this.LocalMatrices[nodeID].Clear());
		}

		public IGlobalMatrix Copy()
		{
			var copy = new DistributedOverlappingMatrix<TMatrix>(Environment, indexer, Format, checkCompatibleVector, checkCompatibleMatrix);
			Environment.DoPerNode(nodeID => copy.LocalMatrices[nodeID] = (TMatrix)this.LocalMatrices[nodeID].Copy());
			return copy;
		}

		public void LinearCombinationIntoThis(double thisCoefficient, IGlobalMatrix otherMatrix, double otherCoefficient)
		{
			DistributedOverlappingMatrix<TMatrix> distributedOther = checkCompatibleMatrix(otherMatrix);
			Action<int> localOperation = nodeID =>
			{
				TMatrix thisSubdomainMatrix = this.LocalMatrices[nodeID];
				TMatrix otherSubdomainMatrix = distributedOther.LocalMatrices[nodeID];
				thisSubdomainMatrix.LinearCombinationIntoThis(thisCoefficient, otherSubdomainMatrix, otherCoefficient);
			};
			Environment.DoPerNode(localOperation);
		}

		public void MultiplyVector(IGlobalVector input, IGlobalVector output)
		{
			DistributedOverlappingVector distributedInput = checkCompatibleVector(input);
			DistributedOverlappingVector distributedOutput = checkCompatibleVector(output);
			MultiplyVector(distributedInput, distributedOutput);
		}

		public void MultiplyVector(DistributedOverlappingVector input, DistributedOverlappingVector output)
		{
			checkCompatibleVector(input);
			checkCompatibleVector(output);

			Action<int> multiplyLocal = nodeID =>
			{
				TMatrix localA = this.LocalMatrices[nodeID];
				Vector localX = input.LocalVectors[nodeID];
				Vector localY = output.LocalVectors[nodeID];
				localA.MultiplyIntoResult(localX, localY);
			};
			Environment.DoPerNode(multiplyLocal);

			output.SumOverlappingEntries();
		}

		public void ScaleIntoThis(double coefficient)
		{
			Environment.DoPerNode(nodeID => this.LocalMatrices[nodeID].ScaleIntoThis(coefficient));
		}
	}
}
