using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.LinearAlgebra.Distributed
{
    public interface IDistributedVector
    {
        IDistributedIndexer Indexer { get; }

        void AddIntoThis(IDistributedVector otherVector)
            => AxpyIntoThis(otherVector, +1);

        void AxpyIntoThis(IDistributedVector otherVector, double otherCoefficient)
            => LinearCombinationIntoThis(1.0, otherVector, otherCoefficient);

        IDistributedVector Copy()
        {
            IDistributedVector clone = CreateZeroVectorWithSameFormat();
            clone.CopyFrom(this);
            return clone;
        }

        void CopyFrom(IDistributedVector other);

        IDistributedVector CreateZeroVectorWithSameFormat();

        double DotProduct(IDistributedVector otherVector);

        void LinearCombinationIntoThis(double thisCoefficient, IDistributedVector otherVector, double otherCoefficient);

        void SubtractIntoThis(IDistributedVector otherVector)
            => AxpyIntoThis(otherVector, -1);
    }
}
