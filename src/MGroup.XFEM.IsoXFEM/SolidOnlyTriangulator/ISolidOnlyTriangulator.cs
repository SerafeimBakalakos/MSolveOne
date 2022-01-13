namespace MGroup.XFEM.IsoXFEM
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.Geometry.ConformingMesh;

	public interface ISolidOnlyTriangulator : IConformingTriangulator
	{
		public Vector ElementNodalLevelSetValues { get; set; }

	}
}
