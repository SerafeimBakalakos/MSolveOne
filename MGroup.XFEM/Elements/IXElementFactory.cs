using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Elements
{
    public interface IXElementFactory<TElement> where TElement: class, IXFiniteElement
    {
        TElement CreateElement(int id, CellType cellType, IReadOnlyList<XNode> nodes);
    }
}
