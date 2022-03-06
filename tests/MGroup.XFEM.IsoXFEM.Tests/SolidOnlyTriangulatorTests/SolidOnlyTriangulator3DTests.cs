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
	using MGroup.XFEM.Geometry.Primitives;

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
			var subtetrahedraComputed=triangulator.CreateSubTetrahedra(element);
			var vertices = new List<double[]>();
			vertices.AddRange(new double[][] { new double[] { -1,-1,-1}, new double[] { +1, -1, -1 } ,
											   new double[] {+ 1,+1,-1}, new double[] { -1, +1, -1 },
											   new double[] { -1,-1,+1}, new double[] { +1, -1, +1 },
											   new double[] { +1, +1, +1}, new double[] { -1, +1, +1 },
											   new double[] { 0,0,-1}, new double[] { 0, 0, +1 },
											   new double[] { 0,-1,0}, new double[] { 0, +1, 0 },
											   new double[] { -1, 0, 0 }, new double[] { 1, 0, 0 },
											   new double[] { 0, 0, 0 }});
			var subtetrahedraExpected = new IsoXfemElementSubtetrahedon3D[]
										{ new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[0],vertices[1],vertices[8],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[1],vertices[2],vertices[8],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[2],vertices[3],vertices[8],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[3],vertices[0],vertices[8],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[4],vertices[5],vertices[9],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[5],vertices[6],vertices[9],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[6],vertices[7],vertices[9],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[7],vertices[4],vertices[9],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[0],vertices[1],vertices[10],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[0],vertices[4],vertices[10],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[4],vertices[5],vertices[10],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[1],vertices[5],vertices[10],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[2],vertices[3],vertices[11],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[2],vertices[6],vertices[11],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[6],vertices[7],vertices[11],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[3],vertices[7],vertices[11],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[3],vertices[0],vertices[12],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[3],vertices[7],vertices[12],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[7],vertices[4],vertices[12],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[0],vertices[4],vertices[12],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[1],vertices[2],vertices[13],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[1],vertices[5],vertices[13],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[5],vertices[6],vertices[13],vertices[14])),
										  new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(vertices[2],vertices[6],vertices[13],vertices[14]))};
			Assert.Equal(subtetrahedraExpected.Length, subtetrahedraComputed.Length);
			int num = 0;
			foreach (var tet4 in subtetrahedraComputed)
			{
				for (int i = 0; i < tet4.VerticesNatural.Count; i++)
				{
					Assert.Equal(subtetrahedraExpected[num].VerticesNatural[i][0], tet4.VerticesNatural[i][0]);
					Assert.Equal(subtetrahedraExpected[num].VerticesNatural[i][1], tet4.VerticesNatural[i][1]);
					Assert.Equal(subtetrahedraExpected[num].VerticesNatural[i][2], tet4.VerticesNatural[i][2]);
				}
				num++;
			}


		}
    }
}
