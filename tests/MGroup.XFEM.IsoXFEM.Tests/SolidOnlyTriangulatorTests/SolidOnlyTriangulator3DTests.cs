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
	using MGroup.LinearAlgebra.Vectors;

	public class SolidOnlyTriangulator3DTests
	{
		[Fact]
		private void CreateSubTetrahedraTest()
		{
			var geometry = new GeometryProperties(2, 2, 2, new int[] { 2, 2, 2 });
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
			Vector nodalStrainEnergyDensity = Vector.CreateZero((geometry.NumberOfElementsX + 1) * (geometry.NumberOfElementsY + 1) * (geometry.NumberOfElementsZ + 1));
			for (int i = 0; i < 9; i++)
			{
				nodalStrainEnergyDensity[i] = -50;
				nodalStrainEnergyDensity[i + 9] = 100;
				nodalStrainEnergyDensity[i + 18] = -200;
			}
			var triangulator = new SolidOnlyTriangulator3D();
			triangulator.NodalLevelSetModel = nodalStrainEnergyDensity;
			triangulator.ElementNodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, 100, 100, -50, -50, 100, 100, -50 });
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
			subtetrahedraExpected[0].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, 100, 25, 25 });
			subtetrahedraExpected[1].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, 100, 25, 25 });
			subtetrahedraExpected[2].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, -50, 25, 25 });
			subtetrahedraExpected[3].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, -50, 25, 25 });
			subtetrahedraExpected[4].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, 100, 25, 25 });
			subtetrahedraExpected[5].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, 100, 25, 25 });
			subtetrahedraExpected[6].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, -50, 25, 25 });
			subtetrahedraExpected[7].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, -50, 25, 25 });
			subtetrahedraExpected[8].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, 100, 25, 25 });
			subtetrahedraExpected[9].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, -50, 25, 25 });
			subtetrahedraExpected[10].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, 100, 25, 25 });
			subtetrahedraExpected[11].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, 100, 25, 25 });
			subtetrahedraExpected[12].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, -50, 25, 25 });
			subtetrahedraExpected[13].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, 100, 25, 25 });
			subtetrahedraExpected[14].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, -50, 25, 25 });
			subtetrahedraExpected[15].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, -50, 25, 25 });
			subtetrahedraExpected[16].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, -50, -50, 25 });
			subtetrahedraExpected[17].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50,-50, -50, 25 });
			subtetrahedraExpected[18].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, -50, -50, 25 });
			subtetrahedraExpected[19].NodalLevelSetValues = Vector.CreateFromArray(new double[] { -50, -50, -50, 25 });
			subtetrahedraExpected[20].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, 100, 100, 25 });
			subtetrahedraExpected[21].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, 100, 100, 25 });
			subtetrahedraExpected[22].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, 100, 100, 25 });
			subtetrahedraExpected[23].NodalLevelSetValues = Vector.CreateFromArray(new double[] { 100, 100, 100, 25 });
			subtetrahedraExpected[0].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[1].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.solidSubTet4;
			subtetrahedraExpected[2].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[3].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[4].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[5].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.solidSubTet4;
			subtetrahedraExpected[6].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[7].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[8].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[9].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[10].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[11].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.solidSubTet4;
			subtetrahedraExpected[12].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[13].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.solidSubTet4;
			subtetrahedraExpected[14].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[15].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[16].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[17].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[18].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[19].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4;
			subtetrahedraExpected[20].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.solidSubTet4;
			subtetrahedraExpected[21].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.solidSubTet4;
			subtetrahedraExpected[22].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.solidSubTet4;
			subtetrahedraExpected[23].PhaseOfSubTet4 = IsoXfemElementSubtetrahedon3D.Phase.solidSubTet4;
			num = 0;
			foreach (var tet4 in subtetrahedraComputed)
			{
				for (int i = 0; i < tet4.NodalLevelSetValues.Length; i++)
				{
					Assert.Equal(subtetrahedraExpected[num].NodalLevelSetValues[i], tet4.NodalLevelSetValues[i]);
				}
				Assert.Equal(subtetrahedraExpected[num].PhaseOfSubTet4, tet4.PhaseOfSubTet4);
				num++;
			}
		}
	}
}
