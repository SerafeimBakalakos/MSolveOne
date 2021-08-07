using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Elements;

//TODO: Perhaps it should also store it sign, store Gauss points and interpolate/extrapolate
//TODO: Simplify the conversions Natural <-> Cartesian
namespace MGroup.XFEM.Geometry.ConformingMesh
{
    public interface IElementSubcell
    {
        CellType CellType { get; }

        IList<double[]> VerticesNatural { get; }

        double[] FindCentroidNatural();

        (double[] centroid, double bulkSize) FindCentroidAndBulkSizeCartesian(IXFiniteElement parentElement);

        IList<double[]> FindVerticesCartesian(IXFiniteElement parentElement);
    }
}
