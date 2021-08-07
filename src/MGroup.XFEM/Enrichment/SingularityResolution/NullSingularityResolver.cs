using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;

namespace MGroup.XFEM.Enrichment.SingularityResolution
{
    public class NullSingularityResolver : ISingularityResolver
    {
        public HashSet<XNode> FindStepEnrichedNodesToRemove(IEnumerable<XNode> stepNodes, IXGeometryDescription boundary)
        {
            return new HashSet<XNode>();
        }
    }
}
