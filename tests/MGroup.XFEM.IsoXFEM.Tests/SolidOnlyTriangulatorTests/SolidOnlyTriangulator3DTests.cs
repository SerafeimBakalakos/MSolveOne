namespace MGroup.XFEM.IsoXFEM.Tests.SolidOnlyTriangulatorTests
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.XFEM.Elements;
	using MGroup.XFEM.IsoXFEM.IsoXfemElements;
	using MGroup.XFEM.IsoXFEM.MeshGeneration;
	using MGroup.XFEM.Materials.Duplicates;
	using MGroup.XFEM.IsoXFEM.SolidOnlyTriangulator;
	using Xunit;

	public class SolidOnlyTriangulator3DTests
	{
		[Fact]
		private void CreateSubTetrahedraTest()
		{
			var geometry = new GeometryProperties(20, 20, 20, new int[] { 2, 2, 2 });
			var material = new ElasticMaterial3D();
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			IMeshGeneration model = new MeshGeneration3D(material, geometry);
			var nodes = model.CreateNodes();
			var element= new IsoXfemElement3D(0, material, geometry, new[]
			{
				nodes[0],
				nodes[9],
				nodes[12],
				nodes[3],
				nodes[1],
				nodes[10],
				nodes[13],
				nodes[4]
			});
			var triangulator = new SolidOnlyTriangulator3D();
			var subtetrahedra=triangulator.CreateSubTetrahedra(element);
		}
	}
}
