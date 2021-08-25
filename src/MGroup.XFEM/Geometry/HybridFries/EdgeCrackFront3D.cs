using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;

namespace MGroup.XFEM.Geometry.HybridFries
{
    public class EdgeCrackFront3D : ICrackFront3D
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="activeTips">
        /// Must be in order and the edges must be consistent with the normal vectors of the triangles.
        /// </param>
        public EdgeCrackFront3D(List<Vertex3D> vertices, List<Vertex3D> activeTips)
        {
            this.Vertices = vertices;

            //TODO: perhaps I can determine the correct order of vertices, without the user having to take care of it:
            //      I should take the same edges as the ones found in triangles containing tip vertices.

            CoordinateSystems = new List<CrackFrontSystem3D>();
            throw new NotImplementedException();
            //for (int v = 0; v < Vertices.Count; ++v)
            //{
            //    var system = new CrackFrontSystem3D(Vertices[v], frontEdges);
            //    CoordinateSystems.Add(system);
            //}
        }

        public List<Vertex3D> Vertices { get; }

        public List<CrackFrontSystem3D> CoordinateSystems { get; }

        public List<Edge3D> Edges => throw new NotImplementedException();

        public Submesh3D UpdateGeometry(CrackFrontGrowth frontGrowth)
        {
            throw new NotImplementedException();
        }
    }
}
