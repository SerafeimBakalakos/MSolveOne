using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions
{
    public class MatrixProduct : IVectorMultipliable
    {
        private readonly IVectorMultipliable[] matrices;

        public MatrixProduct(params IVectorMultipliable[] individualMatrices)
        {
            this.matrices = individualMatrices;
        }

        public int NumColumns => matrices[matrices.Length - 1].NumColumns;

        public int NumRows => matrices[0].NumRows;

        public IVector Multiply(IVectorView lhsVector)
        {
            var result = (IVector)lhsVector;
            for (int i = matrices.Length -1; i >= 0; --i)
            {
                result = matrices[i].Multiply(result);
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
