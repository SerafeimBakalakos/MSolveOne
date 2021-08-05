using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Elements;

namespace MGroup.XFEM.Entities
{
    public interface IXModel : IModel
    {
        List<XNode> XNodes { get; }

        IEnumerable<IXFiniteElement> EnumerateElements();

        void Initialize();

        void Update(Dictionary<int, Vector> subdomainFreeDisplacements);
    }
}
