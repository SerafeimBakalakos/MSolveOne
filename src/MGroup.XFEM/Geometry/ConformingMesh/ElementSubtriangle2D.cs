using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Interpolation;

//TODO: Perhaps it should also store it sign, store Gauss points and interpolate/extrapolate
//TODO: Simplify the conversions Natural <-> Cartesian
namespace MGroup.XFEM.Geometry.ConformingMesh
{
    public class ElementSubtriangle2D : IElementSubcell
    {
        public ElementSubtriangle2D(Triangle2D triangleNatural)
        {
            VerticesNatural = triangleNatural.Vertices;
        }

        public CellType CellType => CellType.Tri3;

        public IList<double[]> VerticesNatural { get; }

        public double[] FindCentroidNatural() => Utilities.FindCentroid(VerticesNatural);

        public (double[] centroid, double bulkSize) FindCentroidAndBulkSizeCartesian(IXFiniteElement parentElement)
        {
            IIsoparametricInterpolation interpolation = parentElement.Interpolation;
            if (interpolation == InterpolationQuad4.UniqueInstance || interpolation == InterpolationTri3.UniqueInstance)
            {
                // The triangle edges will also be linear in Cartesian coordinate system, for Quad4 and Tri3 elements 
                IList<double[]> vertices = FindVerticesCartesian(parentElement);

                double[] centroid = Utilities.FindCentroid(vertices);
                double area = Utilities.CalcPolygonArea(vertices);
                return (centroid, area);
            }
            else
            {
                //TODO: I need to write the equations. The Jacobian determinant comes into play, 
                //      but at how many points should it be calculated?
                throw new NotImplementedException();
            }
        }

        public IList<double[]> FindVerticesCartesian(IXFiniteElement parentElement)
        {
            IIsoparametricInterpolation interpolation = parentElement.Interpolation;
            IReadOnlyList<XNode> nodes = parentElement.Nodes;
            if (interpolation == InterpolationQuad4.UniqueInstance || interpolation == InterpolationTri3.UniqueInstance)
            {
                var verticesCartesian = new double[3][];
                for (int v = 0; v < 3; ++v)
                {
                    verticesCartesian[v] = interpolation.TransformNaturalToCartesian(nodes, VerticesNatural[v]);
                }
                return verticesCartesian;
            }
            else
            {
                //TODO: I should probably return a whole element in this case. E.g. Tri6.
                throw new NotImplementedException();
            }
        }
    }
}
