using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.LSM
{
    public interface ILsmElementInteraction
    {
        (RelativePositionCurveElement relativePosition, IntersectionMesh intersectionMesh)
            FindIntersection(IList<int> nodeIDs, List<double[]> nodeCoords, List<double> nodeLevelSets);
    }
}
