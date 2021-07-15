using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.LinearAlgebra.Distributed
{
    public interface IDistributedMatrix
    {
        IDistributedIndexer Indexer { get; }

        void Multiply(IDistributedVector lhs, IDistributedVector rhs);
    }
}
