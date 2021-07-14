using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Exceptions;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions
{
    public class MatrixLinearCombination : IVectorMultipliable
    {
        private readonly List<double> coefficients = new List<double>();
        private readonly List<IVectorMultipliable> matrices = new List<IVectorMultipliable>();

        public MatrixLinearCombination()
        {
        }

        public int NumColumns => matrices[matrices.Count - 1].NumColumns;

        public int NumRows => matrices[0].NumRows;

        public void Include(double coefficient, IVectorMultipliable matrix)
        {
            //Preconditions.CheckSameMatrixDimensions(matrices[matrices.Count - 1], matrix);
            if (matrices.Count > 0)
            {
                bool sameDimensions = matrices[0].NumRows == matrix.NumRows;
                sameDimensions &= matrices[0].NumColumns == matrix.NumColumns;
                if (!sameDimensions)
                {
                    throw new NonMatchingDimensionsException("The new matrix must have the same dimensions as the current ones");
                }
            }
            coefficients.Add(coefficient);
            matrices.Add(matrix);
        }

        public IVector Multiply(IVectorView lhsVector)
        {
            IVector result = matrices[0].Multiply(lhsVector);
            result.ScaleIntoThis(coefficients[0]);
            for (int i = 1; i < matrices.Count; ++i)
            {
                result.AxpyIntoThis(matrices[i].Multiply(lhsVector), coefficients[i]);
            }
            return result;
        }

        public void Multiply(IVectorView lhsVector, IVector rhsVector)
        {
            IVector result = Multiply(lhsVector);
            rhsVector.CopyFrom(result);
        }
    }
}
