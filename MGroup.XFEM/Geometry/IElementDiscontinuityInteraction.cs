using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Elements;

namespace MGroup.XFEM.Geometry
{
    public interface IElementDiscontinuityInteraction
    {
        //TODO: Perhaps this should be included into RelativePositionCurveElement
        /// <summary>
        /// Examples of such boundaries: crack tips in 2D, crack fronts in 3D.
        /// </summary>
        bool BoundaryOfGeometryInteractsWithElement { get; } 

        IXFiniteElement Element { get; }

        int ParentGeometryID { get; }

        RelativePositionCurveElement RelativePosition { get; }

        IIntersectionMesh ApproximateGlobalCartesian();

        /// <summary>
        /// The weights of the returned <see cref="GaussPoint"/>s include the determinant of the Jacobian from the
        /// natural system of the element to the global cartesian system.
        /// </summary>
        /// <param name="order"></param>
        IReadOnlyList<GaussPoint> GetBoundaryIntegrationPoints(int order);

        /// <summary>
        /// Normal vectors of the discontinuity at each integration point. These vectors refer to the global coordinate system.
        /// They are in the same order as the <see cref="GaussPoint"/> objects in <see cref="GetBoundaryIntegrationPoints(int)"/>.
        /// </summary>
        /// <param name="order"></param>
        IReadOnlyList<double[]> GetNormalsAtBoundaryIntegrationPoints(int order);

        IList<double[]> GetVerticesForTriangulation();
    }
}
