using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.Elements;
using MGroup.LinearAlgebra.Distributed;

namespace MGroup.XFEM.Entities
{
    public interface IXModel : IModel
    {
        Dictionary<int, XNode> Nodes { get; }

        IEnumerable<IXFiniteElement> EnumerateElements();

        void Initialize();

        void Update(IGlobalVector solutionFreeDofs);
    }
}
