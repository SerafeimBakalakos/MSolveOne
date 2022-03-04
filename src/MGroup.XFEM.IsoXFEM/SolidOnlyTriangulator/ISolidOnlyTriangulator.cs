namespace MGroup.XFEM.IsoXFEM
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.Geometry;
	using MGroup.XFEM.Geometry.ConformingMesh;

	public interface ISolidOnlyTriangulator : IConformingTriangulator
	{
		public Vector NodalLevelSetModel { get; set; }
		public Vector ElementNodalLevelSetValues { get; set; }
		public IXGeometryDescription LevelSetDescription { get; set; }
	}
}
