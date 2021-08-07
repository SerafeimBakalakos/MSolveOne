using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Geometry.Tolerances;

//TODO: Allow the option to specify the minimum triangle area.
namespace MGroup.XFEM.Geometry.ConformingMesh
{
    public interface IConformingTriangulator
    {
        IElementSubcell[] FindConformingMesh(IXFiniteElement element,
            IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance);
    }
}
