using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Materials;

namespace MGroup.XFEM.Elements
{
    public interface IXStructuralMultiphaseElement : IXMultiphaseElement
    {
        IStructuralMaterialField MaterialField { get; }
    }
}
