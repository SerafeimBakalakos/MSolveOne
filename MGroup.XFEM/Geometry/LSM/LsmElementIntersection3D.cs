using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Integration;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;

namespace MGroup.XFEM.Geometry.LSM
{
    /// <summary>
    /// A surface resulting from the intersection of a parent surface with a 3D finite element.
    /// Degenerate cases are also possible: null, single point or single curve.
    /// </summary>
    public class LsmElementIntersection3D : IElementDiscontinuityInteraction
    {
        private readonly IIntersectionMesh intersectionMeshNatural;

        public LsmElementIntersection3D(int parentGeometryID, RelativePositionCurveElement relativePosition, 
            IXFiniteElement element, IIntersectionMesh intersectionMeshNatural)
        {
            this.ParentGeometryID = parentGeometryID;
            this.RelativePosition = relativePosition;
            this.Element = element;
            this.intersectionMeshNatural = intersectionMeshNatural;
        }

        public bool BoundaryOfGeometryInteractsWithElement => false;

        public int ParentGeometryID { get; }

        public RelativePositionCurveElement RelativePosition { get; }

        public IXFiniteElement Element { get; } //TODO: Perhaps this should be defined in the interface

        public IIntersectionMesh ApproximateGlobalCartesian()
        {
            return intersectionMeshNatural.MapToOtherSpace(
                vertexNatural => Element.Interpolation.TransformNaturalToCartesian(Element.Nodes, vertexNatural));
        }

        //TODO: Perhaps a dedicated IBoundaryIntegration component is needed,
        //      along with dedicated concrete integrations for triangles, quads, etc
        public IReadOnlyList<GaussPoint> GetBoundaryIntegrationPoints(int order)
        {
            if ((((IElementType)Element).CellType != CellType.Hexa8) && (((IElementType)Element).CellType != CellType.Tet4))
            {
                throw new NotImplementedException();
            }

            // Conforming surfaces intersect 2 elements, thus the integral will be computed twice. Halve the weights to avoid 
            // obtaining double the value of the integral.
            double weightModifier = 1.0;
            if (RelativePosition == RelativePositionCurveElement.Conforming) weightModifier = 0.5;

            var integrationPoints = new List<GaussPoint>();
            IList<double[]> allVertices = intersectionMeshNatural.Vertices;
            foreach ((CellType cellType, int[] cellConnectivity) in intersectionMeshNatural.Cells)
            {
                if (cellType == CellType.Tri3)
                {
                    // Vertices of triangle in natural system
                    var verticesNatural = new double[][]
                    {
                        allVertices[cellConnectivity[0]], allVertices[cellConnectivity[1]], allVertices[cellConnectivity[2]]
                    };

                    // Vertices of triangle in cartesian system
                    var verticesCartesian = new double[3][];
                    for (int v = 0; v < 3; ++v)
                    {
                        verticesCartesian[v] = Element.Interpolation.TransformNaturalToCartesian(
                            Element.Nodes, verticesNatural[v]);
                    }

                    // Determinant of jacobian from auxiliary system of triangle to global cartesian system.
                    // This is possible because the mappings auxiliary -> natural and natural -> cartesian are both affine.
                    // Therefore the normalized triangle in auxiliary system will be projected onto a triangle in global 
                    // cartesian system.
                    var side0 = new double[3];
                    for (int i = 0; i < 3; ++i)
                    {
                        side0[i] = verticesCartesian[1][i] - verticesCartesian[0][i];
                    }
                    var side1 = new double[3];
                    for (int i = 0; i < 3; ++i)
                    {
                        side1[i] = verticesCartesian[2][i] - verticesCartesian[0][i];
                    }
                    double triangleArea = 0.5 * side0.CrossProduct(side1).Norm2();
                    double detJAuxiliaryNatural = 2 * triangleArea;

                    TriangleQuadratureSymmetricGaussian quadrature = ChooseQuadrature(order);
                    foreach (GaussPoint gpAuxiliary in quadrature.IntegrationPoints)
                    {
                        var shapeFuncs = new double[3];
                        shapeFuncs[0] = 1 - gpAuxiliary.Coordinates[0] - gpAuxiliary.Coordinates[1];
                        shapeFuncs[1] = gpAuxiliary.Coordinates[0];
                        shapeFuncs[2] = gpAuxiliary.Coordinates[1];
                        var gpNatural = new double[3];
                        for (int n = 0; n < shapeFuncs.Length; ++n)
                        {
                            for (int i = 0; i < 3; ++i)
                            {
                                gpNatural[i] += shapeFuncs[n] * verticesNatural[n][i];
                            }
                        }

                        double weight = gpAuxiliary.Weight * detJAuxiliaryNatural * weightModifier;
                        integrationPoints.Add(new GaussPoint(gpNatural, weight));
                    }

                }
                else
                {
                    throw new NotImplementedException();
                }
            }
            return integrationPoints;
        }

        public IReadOnlyList<double[]> GetNormalsAtBoundaryIntegrationPoints(int order)
        {
            // Cartesian coordinates of vertices
            var verticesCartesian = new List<double[]>(intersectionMeshNatural.Vertices.Count);
            foreach (double[] vertexNatural in intersectionMeshNatural.Vertices)
            {
                verticesCartesian.Add(Element.Interpolation.TransformNaturalToCartesian(Element.Nodes, vertexNatural));
            }

            // Num points per triangle
            IQuadrature quadrature2D = ChooseQuadrature(order);
            int numGaussPointsPerCell = quadrature2D.IntegrationPoints.Count;

            // Find normal vectors of each triangle
            var allNormals = new List<double[]>();
            foreach ((CellType cellType, int[] verticesOfCell) in intersectionMeshNatural.Cells)
            {
                Debug.Assert(cellType == CellType.Tri3);

                double[] vertex0 = verticesCartesian[verticesOfCell[0]];
                double[] vertex1 = verticesCartesian[verticesOfCell[1]];
                double[] vertex2 = verticesCartesian[verticesOfCell[2]];
                double[] normalVector = CalcNormalOfPlaneThrough(vertex0, vertex1, vertex2);

                for (int i = 0; i < numGaussPointsPerCell; ++i)
                {
                    allNormals.Add(normalVector);
                }
            }

            return allNormals;
        }

        public IList<double[]> GetVerticesForTriangulation()
        {
            return intersectionMeshNatural.Vertices;
        }

        private double[] CalcNormalOfPlaneThrough(double[] point0, double[] point1, double[] point2)
        {
            // Find 2 vectors parallel to plane
            var parallelVector0 = Vector.CreateZero(3);
            var parallelVector1 = Vector.CreateZero(3);
            for (int d = 0; d < 3; ++d)
            {
                parallelVector0[d] = point1[d] - point0[d];
                parallelVector1[d] = point2[d] - point0[d];
            }

            // The normal vector is their cross-product
            Vector normal = parallelVector0.CrossProduct(parallelVector1);
            normal.ScaleIntoThis(1.0 / normal.Norm2());

            return normal.RawData;
        }

        private TriangleQuadratureSymmetricGaussian ChooseQuadrature(int order)
        {
            if (order <= 1) return TriangleQuadratureSymmetricGaussian.Order1Point1;
            else if (order == 2) return TriangleQuadratureSymmetricGaussian.Order2Points3;
            else if (order == 3) return TriangleQuadratureSymmetricGaussian.Order3Points4;
            else throw new NotImplementedException();
        }
    }
}
