using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

//TODO: Also provide the calculation of normal vector through point/node
//TODO: Remove dependencies from XElement and XNode. Geometry classes interact with geometric shapes. Meaning that, this object
//      should interact with other element and node classes as well.
namespace MGroup.XFEM.Geometry
{
    public interface IXGeometryDescription
    {
        //TODO: Remove this. IDs are for model entities, such as cracks and phases. The underlying geometry classes only
        //      provide the necessary math. At most they contain some state (e.g. level sets over a mesh; not necessarily XNodes)
        //      which can also be mutated. 
        int ID { get; }

        IElementDiscontinuityInteraction Intersect(IXFiniteElement element);

        double SignedDistanceOf(XNode node);

        double SignedDistanceOf(XPoint point);

    }
}
