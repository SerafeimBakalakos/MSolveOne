using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Matrices.Builders;
using MGroup.LinearAlgebra.Reduction;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class DokMatrixAdapter : IMatrix
	{
		public DokMatrixAdapter(DokSymmetric dokMatrix)
		{
			DokMatrix = dokMatrix;
		}

		public double this[int rowIdx, int colIdx] => DokMatrix[rowIdx, colIdx];

		public DokSymmetric DokMatrix { get; }

		public int NumColumns => DokMatrix.NumColumns;

		public int NumRows => DokMatrix.NumRows;

		public IMatrix Axpy(IMatrixView otherMatrix, double otherCoefficient)
		{
			throw new NotImplementedException();
		}

		public void AxpyIntoThis(IMatrixView otherMatrix, double otherCoefficient)
		{
			throw new NotImplementedException();
		}

		public void Clear()
		{
			throw new NotImplementedException();
		}

		public IMatrix Copy(bool copyIndexingData = false)
		{
			throw new NotImplementedException();
		}

		public Matrix CopyToFullMatrix()
		{
			throw new NotImplementedException();
		}

		public IMatrix DoEntrywise(IMatrixView matrix, Func<double, double, double> binaryOperation)
		{
			throw new NotImplementedException();
		}

		public void DoEntrywiseIntoThis(IMatrixView matrix, Func<double, double, double> binaryOperation)
		{
			throw new NotImplementedException();
		}

		public IMatrix DoToAllEntries(Func<double, double> unaryOperation)
		{
			throw new NotImplementedException();
		}

		public void DoToAllEntriesIntoThis(Func<double, double> unaryOperation)
		{
			throw new NotImplementedException();
		}

		public bool Equals(IIndexable2D other, double tolerance = 1E-13)
		{
			throw new NotImplementedException();
		}

		public Vector GetColumn(int colIndex)
		{
			throw new NotImplementedException();
		}

		public Vector GetRow(int rowIndex)
		{
			throw new NotImplementedException();
		}

		public IMatrix GetSubmatrix(int[] rowIndices, int[] colIndices)
		{
			throw new NotImplementedException();
		}

		public IMatrix GetSubmatrix(int rowStartInclusive, int rowEndExclusive, int colStartInclusive, int colEndExclusive)
		{
			throw new NotImplementedException();
		}

		public IMatrix LinearCombination(double thisCoefficient, IMatrixView otherMatrix, double otherCoefficient)
		{
			throw new NotImplementedException();
		}

		public void LinearCombinationIntoThis(double thisCoefficient, IMatrixView otherMatrix, double otherCoefficient)
		{
			throw new NotImplementedException();
		}

		public IVector Multiply(IVectorView vector, bool transposeThis = false)
		{
			throw new NotImplementedException();
		}

		public void MultiplyIntoResult(IVectorView lhsVector, IVector rhsVector, bool transposeThis = false)
		{
			throw new NotImplementedException();
		}

		public Matrix MultiplyLeft(IMatrixView other, bool transposeThis = false, bool transposeOther = false)
		{
			throw new NotImplementedException();
		}

		public Matrix MultiplyRight(IMatrixView other, bool transposeThis = false, bool transposeOther = false)
		{
			throw new NotImplementedException();
		}

		public double Reduce(double identityValue, ProcessEntry processEntry, ProcessZeros processZeros, Finalize finalize)
		{
			throw new NotImplementedException();
		}

		public IMatrix Scale(double scalar)
		{
			throw new NotImplementedException();
		}

		public void ScaleIntoThis(double scalar)
		{
			throw new NotImplementedException();
		}

		public void SetEntryRespectingPattern(int rowIdx, int colIdx, double value)
		{
			DokMatrix[rowIdx, colIdx] = value;
		}

		public IMatrix Transpose()
		{
			throw new NotImplementedException();
		}
	}
}
