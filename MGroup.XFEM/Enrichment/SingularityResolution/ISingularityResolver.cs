using System.Collections.Generic;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;

namespace MGroup.XFEM.Enrichment.SingularityResolution
{
    public interface ISingularityResolver
    {
        HashSet<XNode> FindStepEnrichedNodesToRemove(IEnumerable<XNode> stepNodes, IXGeometryDescription boundary);
    }
}
