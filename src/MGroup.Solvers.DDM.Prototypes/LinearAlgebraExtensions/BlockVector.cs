using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Reduction;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions
{
	public class BlockVector : IVector
	{
		private readonly Dictionary<int, int> lengthPerBlock;
		private readonly int numBlocks;

		public BlockVector(IDictionary<int, int> lengthPerBlock)
		{
			if (!Utilities.AreIndices(lengthPerBlock.Keys))
			{
				throw new ArgumentException(
					"The keys of the provided dictionary must be indices, namely be unique and belong to [0, count)");
			}


			this.lengthPerBlock = new Dictionary<int, int>();
			foreach (var blockIdxLengthPair in lengthPerBlock)
			{
				int blockIdx = blockIdxLengthPair.Key;
				int blockLength = blockIdxLengthPair.Value;
				++this.numBlocks;
				this.lengthPerBlock[blockIdx] = blockLength;
				this.Length += blockLength;

			}

			this.Blocks = new Vector[numBlocks];
		}

		public BlockVector(Vector singleVector) : this(new Dictionary<int, int> { { 0, singleVector.Length } })
		{
			this.Blocks[0] = singleVector;
		}

		public double this[int index] => throw new NotImplementedException();

		public Vector[] Blocks { get; }

		public int Length { get; }

		public int[][] Multiplicities { get; set; }

		public void AddBlock(int blockIdx, Vector block)
		{
			Blocks[blockIdx] = block;
		}

		public void AddIntoThisNonContiguouslyFrom(int[] thisIndices, IVectorView otherVector, int[] otherIndices)
		{
			throw new NotImplementedException();
		}

		public void AddIntoThisNonContiguouslyFrom(int[] thisIndices, IVectorView otherVector)
		{
			throw new NotImplementedException();
		}

		public IVector Axpy(IVectorView otherVector, double otherCoefficient)
		{
			IVector result = this.Copy();
			result.AxpyIntoThis(otherVector, otherCoefficient);
			return result;
		}

		public void AxpyIntoThis(IVectorView otherVector, double otherCoefficient)
		{
			var otherCasted = (BlockVector)otherVector;
			for (int b = 0; b < this.Blocks.Length; ++b)
			{
				this.Blocks[b].AxpyIntoThis(otherCasted.Blocks[b], otherCoefficient);
			}
		}

		public void AxpySubvectorIntoThis(int destinationIndex, IVectorView sourceVector, double sourceCoefficient, int sourceIndex, int length)
		{
			throw new NotImplementedException();
		}

		public void Clear()
		{
			for (int b = 0; b < this.Blocks.Length; ++b)
			{
				Blocks[b].Clear();
			}
		}

		IVector IVectorView.Copy(bool copyIndexingData = false) => Copy(copyIndexingData);

		public BlockVector Copy(bool copyIndexingData = false)
		{
			var result = new BlockVector(this.lengthPerBlock);
			result.Multiplicities = this.Multiplicities;
			for (int b = 0; b < this.Blocks.Length; ++b)
			{
				result.Blocks[b] = this.Blocks[b].Copy();
			}
			return result;
		}

		public void CopyFrom(IVectorView sourceVector)
		{
			var otherCasted = (BlockVector)sourceVector;
			this.Multiplicities = otherCasted.Multiplicities;
			for (int b = 0; b < this.Blocks.Length; ++b)
			{
				this.Blocks[b].CopyFrom(otherCasted.Blocks[b]);
			}
		}

		public void CopyNonContiguouslyFrom(int[] thisIndices, IVectorView otherVector, int[] otherIndices)
		{
			throw new NotImplementedException();
		}

		public void CopyNonContiguouslyFrom(IVectorView otherVector, int[] otherIndices)
		{
			throw new NotImplementedException();
		}

		public void CopySubvectorFrom(int destinationIndex, IVectorView sourceVector, int sourceIndex, int length)
		{
			throw new NotImplementedException();
		}

		public double[] CopyToArray() => CopyToFullVector().RawData;

		public Vector CopyToFullVector()
		{
			var result = Vector.CreateZero(this.Length);
			int start = 0;
			for (int b = 0; b < this.Blocks.Length; ++b)
			{
				result.CopySubvectorFrom(start, Blocks[b], 0, Blocks[b].Length);
				start += Blocks[b].Length;
			}

			return result;
		}

		IVector IVectorView.CreateZeroVectorWithSameFormat() => CreateZeroVectorWithSameFormat();

		public BlockVector CreateZeroVectorWithSameFormat()
		{
			var result = new BlockVector(this.lengthPerBlock);
			result.Multiplicities = this.Multiplicities;
			for (int b = 0; b < result.numBlocks; ++b)
			{
				result.Blocks[b] = Vector.CreateZero(result.lengthPerBlock[b]);
			}
			return result;
		}

		public IVector DoEntrywise(IVectorView vector, Func<double, double, double> binaryOperation)
		{
			throw new NotImplementedException();
		}

		public void DoEntrywiseIntoThis(IVectorView otherVector, Func<double, double, double> binaryOperation)
		{
			throw new NotImplementedException();
		}

		IVector IEntrywiseOperableView1D<IVectorView, IVector>.DoToAllEntries(Func<double, double> unaryOperation)
			=> DoToAllEntries(unaryOperation);

		public BlockVector DoToAllEntries(Func<double, double> unaryOperation)
		{
			BlockVector clone = Copy();
			clone.DoToAllEntriesIntoThis(unaryOperation);
			return clone;
		}

		public void DoToAllEntriesIntoThis(Func<double, double> unaryOperation)
		{
			foreach (Vector block in Blocks)
			{
				block.DoToAllEntriesIntoThis(unaryOperation);
			}
		}

		public double DotProduct(IVectorView vector)
		{
			var otherCasted = (BlockVector)vector;
			if (this.numBlocks == 1)
			{
				return this.Blocks[0] * otherCasted.Blocks[0];
			}
			else
			{
				Debug.Assert(this.Multiplicities == otherCasted.Multiplicities);
				double sum = 0.0;
				for (int b = 0; b < numBlocks; ++b)
				{
					for (int i = 0; i < Multiplicities[b].Length; ++i)
					{
						sum += this.Blocks[b][i] * otherCasted.Blocks[b][i] / Multiplicities[b][i];
					}
				}
				return sum;
			}
		}

		public bool Equals(IIndexable1D other, double tolerance = 1E-13)
		{
			if (other is BlockVector otherCasted)
			{
				for (int b = 0; b < this.numBlocks; ++b)
				{
					if (!this.Blocks[b].Equals(otherCasted.Blocks[b], tolerance))
					{
						return false;
					}
				}
				return true;
			}
			else return false;
		}

		public IVector LinearCombination(double thisCoefficient, IVectorView otherVector, double otherCoefficient)
		{
			IVector result = this.Copy();
			result.LinearCombinationIntoThis(thisCoefficient, otherVector, otherCoefficient);
			return result;
		}

		public void LinearCombinationIntoThis(double thisCoefficient, IVectorView otherVector, double otherCoefficient)
		{
			var otherCasted = (BlockVector)otherVector;
			for (int b = 0; b < this.Blocks.Length; ++b)
			{
				this.Blocks[b].LinearCombinationIntoThis(thisCoefficient, otherCasted.Blocks[b], otherCoefficient);
			}
		}

		public IVector MultiplyEntryWise(IVectorView otherVector)
		{
			var result = (BlockVector)this.Copy();
			var otherCasted = (BlockVector)otherVector;
			for (int b = 0; b < this.Blocks.Length; ++b)
			{
				result.Blocks[b].MultiplyEntrywiseIntoThis(otherCasted.Blocks[b]);
			}
			return result;
		}

		public double Norm2() => Math.Sqrt(DotProduct(this));

		public double Reduce(double identityValue, ProcessEntry processEntry, ProcessZeros processZeros, Finalize finalize)
		{
			throw new NotImplementedException();
		}

		public IVector Scale(double scalar)
		{
			IVector result = this.Copy();
			result.ScaleIntoThis(scalar);
			return result;
		}

		public void ScaleIntoThis(double scalar)
		{
			for (int b = 0; b < this.Blocks.Length; ++b)
			{
				this.Blocks[b].ScaleIntoThis(scalar);
			}
		}

		public void Set(int index, double value)
		{
			throw new NotImplementedException();
		}
	}
}
