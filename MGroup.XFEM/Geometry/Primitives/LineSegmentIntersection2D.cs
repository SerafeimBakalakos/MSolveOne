using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.XFEM.Integration;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Integration.Quadratures;

//TODO: remove duplication between this and Line2D & LineSegment2D. Why can't this inherit from LineSegment2D? 
//      Or just use LineSegment2D wrapped in a class about Intersection
namespace MGroup.XFEM.Geometry.Primitives
{
    public class LineSegmentIntersection2D : IElementDiscontinuityInteraction
    {
        /// <summary>
        /// a is the counter-clockwise angle from the global x axis to the local x axis
        /// Transformation matrix from global to local system: Q = [cosa sina; -sina cosa ]
        /// Transformation matrix from local to global system: Q^T = [cosa -sina; sina cosa ]
        /// </summary>
        protected readonly double cosa, sina;

        ///// <summary>
        ///// The coordinates of the global system's origin in the local system
        ///// </summary>
        protected readonly double[] originLocal;

        public LineSegmentIntersection2D(RelativePositionCurveElement pos, double cosa, double sina, double[] originLocal, 
            double startLocalX, double endLocalX)
        {
            Debug.Assert(pos == RelativePositionCurveElement.Intersecting || pos == RelativePositionCurveElement.Conforming);
            this.RelativePosition = pos;
            this.cosa = cosa;
            this.sina = sina;
            this.originLocal = originLocal;
            this.StartLocalX = startLocalX;
            this.EndLocalX = endLocalX;

            this.StartGlobalCartesian = ProjectLocalToGlobal(startLocalX);
            this.EndGlobalCartesian = ProjectLocalToGlobal(endLocalX);
        }

        public LineSegmentIntersection2D(int parentGeometryID, double[] start, double[] end, IXFiniteElement element, 
            RelativePositionCurveElement pos)
        {
            Debug.Assert(pos == RelativePositionCurveElement.Intersecting || pos == RelativePositionCurveElement.Conforming);
            this.ParentGeometryID = parentGeometryID;
            this.StartGlobalCartesian = start;
            this.EndGlobalCartesian = end;
            this.Element = element;
            this.RelativePosition = pos;

            double dx = end[0] - start[0];
            double dy = end[1] - start[1];

            double length = Math.Sqrt(dx * dx + dy * dy);
            this.cosa = dx / length;
            this.sina = dy / length;

            this.originLocal = new double[2];
            this.originLocal[0] = -cosa * start[0] - sina * start[1];
            this.originLocal[1] = sina * start[0] - cosa * start[1];
        }

        public bool BoundaryOfGeometryInteractsWithElement => throw new NotImplementedException();


        public int ParentGeometryID { get; }

        public RelativePositionCurveElement RelativePosition { get; }

        public double[] StartGlobalCartesian { get; }

        public double StartLocalX { get; }

        public double[] EndGlobalCartesian { get; }

        public double EndLocalX { get; }

        public IXFiniteElement Element { get; }

        public IIntersectionMesh ApproximateGlobalCartesian()
        {
            var meshCartesian = new IntersectionMesh3D_OLD();
            meshCartesian.Vertices.Add(StartGlobalCartesian);
            meshCartesian.Vertices.Add(EndGlobalCartesian);
            meshCartesian.Cells.Add((CellType.Line2, new int[] { 0, 1 }));
            return meshCartesian;
        }

        public IReadOnlyList<GaussPoint> GetBoundaryIntegrationPoints(int numPoints)
        {
            // If conforming: halve the weights. Perhaps this can be done in the XElement
            throw new NotImplementedException();
        }

        public IReadOnlyList<double[]> GetNormalsAtBoundaryIntegrationPoints(int order)
        {
            // Orientation of segment and its normal vector
            double dx = EndGlobalCartesian[0] - StartGlobalCartesian[0];
            double dy = EndGlobalCartesian[1] - StartGlobalCartesian[1];
            double length = Math.Sqrt(dx * dx + dy * dy);
            double cosa = dx / length;
            double sina = dy / length;
            double[] normalVector = { -sina, cosa };

            // Prepare as many normal vectors as there are Gauss points
            var quadrature1D = GaussLegendre1D.GetQuadratureWithOrder(order);
            int numGaussPoints = quadrature1D.IntegrationPoints.Count;
            var normalVectors = new double[numGaussPoints][];
            for (int i = 0; i < numGaussPoints; ++i)
            {
                normalVectors[i] = normalVector;
            }

            return normalVectors;
        }

        public IList<double[]> GetVerticesForTriangulation()
        {
            throw new NotImplementedException();
        }

        private double[] ProjectLocalToGlobal(double localX)
        {
            // xGlobal = Q^T * (xLocal - originLocal)
            double dx = localX - originLocal[0];
            double dy = -originLocal[1];
            return new double[2]
            {
                cosa * dx - sina * dy,
                sina * dx + cosa * dy
            };
        }
    }
}
