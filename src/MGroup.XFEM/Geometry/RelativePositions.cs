namespace MGroup.XFEM.Geometry
{
    public enum RelativePositionClosedCurves
    {
        Disjoint, Intersecting, Tangent, Conforming
    }

    //TODO: rename this. It is not only for curves, but also for surfaces
    //TODO: the interaction of the element with the boundary (tip/front) of the geometry (curve/surface respectively) should
    //      be described by this component, instead of needing a separate bool flag (see IElementDiscontinuityInteraction)
    public enum RelativePositionCurveElement 
    {
        /// <summary>
        /// There are no common points.
        /// </summary>
        Disjoint,

        /// <summary>
        /// Degenerate case of <see cref="Intersecting"/>.
        /// In 2D: There is a single common point. 
        /// In 3D: There is a single common curve segment.
        /// </summary>
        Tangent,

        /// <summary>
        /// In 2D: the curve intersects at least 1 element edge, forming a common curve segment.
        /// In 3D: the surface intersects at least 1 element face, forming a common surface segment.
        /// </summary>
        Intersecting,

        /// <summary>
        /// In 2D: An element edge lies on the curve.
        /// In 3D: An element face lies on the surface.
        /// </summary>
        Conforming
    }
}
