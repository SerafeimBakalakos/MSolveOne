using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.LinearAlgebra.Distributed
{
    public interface IDistributedIndexer
    {
        //TODOMPI: Also check that environment is the same
        bool Matches(IDistributedIndexer other);
    }
}
