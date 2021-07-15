using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DDM.Partitioning
{
    public interface IPartitioner
    {
        int NumSubdomainsTotal { get; }

        int GetClusterOfSubdomain(int subdomainID);

        IEnumerable<int> GetNeighboringSubdomains(int subdomainID);

        int GetSubdomainOfElement(int elementID);

        void Partition(IModel model);
    }
}
