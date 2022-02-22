namespace MGroup.XFEM.IsoXFEM.IsoXfemElements
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Reduction;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization.Mesh;
	using MGroup.XFEM.Elements;
	using MGroup.XFEM.Entities;
	using MGroup.XFEM.Geometry;
	using MGroup.XFEM.Geometry.ConformingMesh;
	using MGroup.XFEM.Geometry.Primitives;
	using MGroup.XFEM.Interpolation;

	public class IsoXfemElementSubtetrahedon3D : IElementSubcell
	{
		public IsoXfemElementSubtetrahedon3D(Tetrahedron3D tetraNatural)
		{
			VerticesNatural = tetraNatural.Vertices;
		}

		public Vector NodalLevelSetValues { get; set; }
		public enum Phase
		{
			solidSubTet4,
			voidSubTet4,
			boundarySubTet4
		}
		public Phase PhaseOfSubTet4 { get; set; }
		public void DefinePhaseOfElement()
		{
			for (int i = 0; i < NodalLevelSetValues.Length; i++)
			{
				if (NodalLevelSetValues[i]==0)
				{
					NodalLevelSetValues[i] = NodalLevelSetValues.Max() * 0.0001;
				}
			}
			if (NodalLevelSetValues.Min() > 0)
			{
				PhaseOfSubTet4 = Phase.solidSubTet4;
			}
			else if (NodalLevelSetValues.Max() < 0)
			{
				PhaseOfSubTet4 = Phase.voidSubTet4;
			}
			else
			{
				PhaseOfSubTet4 = Phase.boundarySubTet4;
			}
		}
		public CellType CellType => CellType.Tet4;

		public IList<double[]> VerticesNatural { get; }

		public double[] FindCentroidNatural() => Utilities.FindCentroid(VerticesNatural);

		public (double[] centroid, double bulkSize) FindCentroidAndBulkSizeCartesian(IXFiniteElement parentElement)
		{
			IIsoparametricInterpolation interpolation = parentElement.Interpolation;
			if (interpolation == InterpolationTet4.UniqueInstance || interpolation == InterpolationHexa8.UniqueInstance)
			{
				// The tetrahedron edges will also be linear in Cartesian coordinate system, for Tetra4 and Hexa8 elements 
				IList<double[]> vertices = FindVerticesCartesian(parentElement);
				double[] centroid = Utilities.FindCentroid(vertices);
				double volume = Utilities.CalcTetrahedronVolume(vertices);
				return (centroid, volume);
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
			if (interpolation == InterpolationTet4.UniqueInstance || interpolation == InterpolationHexa8.UniqueInstance)
			{
				var verticesCartesian = new double[4][];
				for (int v = 0; v < 4; ++v)
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
