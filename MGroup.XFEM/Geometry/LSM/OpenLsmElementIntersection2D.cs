using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;

namespace MGroup.XFEM.Geometry.LSM
{
    public class OpenLsmElementIntersection2D : IElementDiscontinuityInteraction
    {
        private readonly IList<double[]> commonPointsNatural;

        public OpenLsmElementIntersection2D(int parentGeometryID, IXFiniteElement element, 
            RelativePositionCurveElement relativePosition, bool tipInteractsWithElement, IList<double[]> commonPointsNatural)
        {
            this.ParentGeometryID = parentGeometryID;
            this.Element = element;
            if (relativePosition == RelativePositionCurveElement.Disjoint)
            {
                throw new ArgumentException("There is no intersection between the curve and element");
            }
            this.RelativePosition = relativePosition;
            this.BoundaryOfGeometryInteractsWithElement = tipInteractsWithElement;
            this.commonPointsNatural = commonPointsNatural;
        }

        public IXFiniteElement Element { get; }

        public int ParentGeometryID { get; }

        public RelativePositionCurveElement RelativePosition { get; }

        public bool BoundaryOfGeometryInteractsWithElement { get; }

        public IIntersectionMesh ApproximateGlobalCartesian()
        {
            throw new NotImplementedException();
        }

        public IReadOnlyList<GaussPoint> GetBoundaryIntegrationPoints(int order)
        {
            throw new NotImplementedException();
        }

        public IReadOnlyList<double[]> GetNormalsAtBoundaryIntegrationPoints(int order)
        {
            // Cartesian coordinates of vertices
            var verticesCartesian = new List<double[]>(commonPointsNatural.Count);
            foreach (double[] vertexNatural in commonPointsNatural)
            {
                verticesCartesian.Add(Element.Interpolation.TransformNaturalToCartesian(Element.Nodes, vertexNatural));
            }

            // Num points per segment
            var quadrature1D = GaussLegendre1D.GetQuadratureWithOrder(order);
            int numGaussPointsPerSegment = quadrature1D.IntegrationPoints.Count;

            // Find normal vectors of each segment
            var allNormals = new List<double[]>();
            for (int c = 0; c < commonPointsNatural.Count; ++c)
            {
                //TODO: It would be safer to find the vertices from the cells, instead of assuming that they are in order.
                double[] startCartesian = verticesCartesian[c];
                double[] endCartesian = verticesCartesian[c + 1];

                IList<double[]> normalsOfSegment =
                    GetNormalVectorsOfSegment(numGaussPointsPerSegment, startCartesian, endCartesian);
                allNormals.AddRange(normalsOfSegment);
            }

            return allNormals;
        }

        public IList<double[]> GetVerticesForTriangulation()
        {
            if (RelativePosition == RelativePositionCurveElement.Intersecting) return commonPointsNatural;
            else return new double[0][];
        }

        private IList<double[]> GetNormalVectorsOfSegment(int numGaussPoints, double[] startCartesian, double[] endCartesian)
        {
            double dx = endCartesian[0] - startCartesian[0];
            double dy = endCartesian[1] - startCartesian[1];
            double length = Math.Sqrt(dx * dx + dy * dy);
            double cosa = dx / length;
            double sina = dy / length;
            double[] normalVector = { -sina, cosa };

            var normalVectors = new double[numGaussPoints][];
            for (int i = 0; i < numGaussPoints; ++i)
            {
                normalVectors[i] = normalVector;
            }

            return normalVectors;
        }
    }
}
