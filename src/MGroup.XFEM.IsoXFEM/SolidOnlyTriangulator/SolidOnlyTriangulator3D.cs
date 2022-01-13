namespace MGroup.XFEM.IsoXFEM.SolidOnlyTriangulator
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.Elements;
	using MGroup.XFEM.Geometry;
	using MGroup.XFEM.Geometry.ConformingMesh;
	using MGroup.XFEM.Geometry.Tolerances;

	public class SolidOnlyTriangulator3D : ISolidOnlyTriangulator
	{
		public Vector ElementNodalLevelSetValues { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }

		public IElementSubcell[] FindConformingMesh(IXFiniteElement element, IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance) => throw new NotImplementedException();
	}
}
