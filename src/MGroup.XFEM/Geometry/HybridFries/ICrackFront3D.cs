using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;

namespace MGroup.XFEM.Geometry.HybridFries
{
    public interface ICrackFront3D
    {
        //TODO: Perhaps this should be stored in the vertices.
        List<CrackFrontSystem3D> CoordinateSystems { get; }

        List<Edge3D> Edges { get; }
        
        List<Vertex3D> Vertices { get; }

        Submesh3D UpdateGeometry(CrackFrontGrowth frontGrowth);
    }
}
