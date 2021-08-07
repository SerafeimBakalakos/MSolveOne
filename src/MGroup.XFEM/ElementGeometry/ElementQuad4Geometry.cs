using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Interpolation;

namespace MGroup.XFEM.ElementGeometry
{
    public class ElementQuad4Geometry : IElementGeometry
    {
        public double CalcBulkSizeCartesian(IReadOnlyList<XNode> nodes) 
            => Utilities.CalcPolygonArea(nodes.Select(n => n.Coordinates).ToArray());

        public double CalcBulkSizeNatural() => 4.0;

        public (ElementEdge[], ElementFace[]) FindEdgesFaces(IReadOnlyList<int> nodeIDs)
        {
            IReadOnlyList<double[]> nodesNatural = InterpolationQuad4.UniqueInstance.NodalNaturalCoordinates;
            var edges = new ElementEdge[4];
            edges[0] = new ElementEdge(0, nodeIDs, nodesNatural, 0, 1);
            edges[1] = new ElementEdge(1, nodeIDs, nodesNatural, 1, 2);
            edges[2] = new ElementEdge(2, nodeIDs, nodesNatural, 2, 3);
            edges[3] = new ElementEdge(3, nodeIDs, nodesNatural, 3, 0);
            return (edges, new ElementFace[0]);
        }
    }
}
