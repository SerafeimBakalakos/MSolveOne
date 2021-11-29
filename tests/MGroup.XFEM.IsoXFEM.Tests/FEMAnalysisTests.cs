namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System.IO;

	using MGroup.LinearAlgebra.Input;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.IsoXFEM.Solvers;
	using MGroup.XFEM.Materials.Duplicates;

	using Xunit;
	public class FEMAnalysisTests
	{		

		[Fact]
		private void AssembleStiffnessMatrix()
		{
			var geometry = new GeometryProperties(30, 10, 1, 3, 1);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var model = new Model(material, geometry);
			model.MakeMesh();
			model.EnumerateDegreesOfFreedom();
			var nodalLoad = new NodalLoad(geometry, EndLoad.BottomEnd);
			Vector rhs = nodalLoad.CalcRHS();
			ISolver solver = new SkylineLdlSolver();
			var femAnalysis = new FEMAnalysis(model, solver, rhs);
			femAnalysis.AssembleStiffnessMatrix();
			Matrix globalStiffnessComputed = femAnalysis.globalStiffness;
			Matrix globalStiffnessExpected= Matrix.CreateFromArray(new double[,] {{ 0.494505494505,   0.178571428571,  0.0549450549451, 0.0137362637363, -0.302197802198, -0.0137362637363, -0.247252747253, -0.178571428571, 0,   0,   0,   0,   0,   0,   0,   0 },
				{0.178571428571, 0.494505494505,  -0.0137362637363, -0.302197802198, 0.0137362637363, 0.0549450549451, -0.178571428571, -0.247252747253, 0,   0,  0,   0,   0,   0,   0,   0, },
				{0.0549450549451,    -0.0137362637363,    0.494505494505,  -0.178571428571, -0.247252747253, 0.178571428571,  -0.302197802198, 0.0137362637363, 0,   0,   0,   0,   0,   0,   0,   0 },
				{0.0137362637363,    -0.302197802198, -0.178571428571, 0.494505494505,  0.178571428571,  -0.247252747253, -0.0137362637363,    0.0549450549451, 0,   0,   0,   0,   0,   0,   0,   0 },
				{-0.302197802198,    0.0137362637363, -0.247252747253, 0.178571428571,  0.989010989011,  0,   0.10989010989,   0,   -0.302197802198, -0.0137362637363,    -0.247252747253, -0.178571428571, 0,   0,   0,   0 },
				{-0.0137362637363,   0.0549450549451, 0.178571428571,  -0.247252747253, 0,   0.989010989011,  0,   -0.604395604396, 0.0137362637363, 0.0549450549451, -0.178571428571, -0.247252747253, 0,   0,   0,   0 },
				{-0.247252747253,    -0.178571428571, -0.302197802198, -0.0137362637363,    0.10989010989,   0,   0.989010989011,  0,   -0.247252747253, 0.178571428571,  -0.302197802198, 0.0137362637363, 0,   0,   0,   0 },
				{ -0.178571428571,   -0.247252747253, 0.0137362637363, 0.0549450549451, 0,   -0.604395604396, 0,   0.989010989011,  0.178571428571,  -0.247252747253, -0.0137362637363,    0.0549450549451, 0,   0,   0,   0},
				{0,  0,   0,   0,   -0.302197802198, 0.0137362637363, -0.247252747253, 0.178571428571,  0.989010989011,  0,   0.10989010989,   0,   -0.302197802198, -0.0137362637363,    -0.247252747253, -0.178571428571 },
				{0,  0,   0,   0,   -0.0137362637363, 0.0549450549451, 0.178571428571,  -0.247252747253, 0,   0.989010989011,  0,   -0.604395604396, 0.0137362637363, 0.0549450549451, -0.178571428571, -0.247252747253 },
				{0,  0,   0,   0,   -0.247252747253, -0.178571428571, -0.302197802198, -0.0137362637363,  0.10989010989,   0,   0.989010989011,  0,   -0.247252747253, 0.178571428571,  -0.302197802198, 0.0137362637363 },
				{0,  0,   0,   0,   -0.178571428571, -0.247252747253, 0.0137362637363, 0.0549450549451, 0,   -0.604395604396, 0,   0.989010989011,  0.178571428571,  -0.247252747253, -0.0137362637363,    0.0549450549451 },
				{0,  0,   0,   0,   0,   0,   0,   0,  -0.302197802198, 0.0137362637363, -0.247252747253, 0.178571428571,  0.494505494505,  -0.178571428571, 0.0549450549451, -0.0137362637363 },
				{0,  0,   0,   0,   0,   0,   0,   0,   -0.0137362637363,    0.0549450549451, 0.178571428571,  -0.247252747253, -0.178571428571, 0.494505494505,  0.0137362637363, -0.302197802198 },
				{0,  0,   0,   0,   0,   0,   0,   0,   -0.247252747253, -0.178571428571, -0.302197802198, -0.0137362637363,    0.0549450549451, 0.0137362637363, 0.494505494505,  0.178571428571 },
				{0,  0,   0,   0,   0,   0,   0,   0,   -0.178571428571, -0.247252747253, 0.0137362637363, 0.0549450549451, -0.0137362637363,    -0.302197802198, 0.178571428571,  0.494505494505 } });
			for (int i = 0; i < globalStiffnessExpected.NumRows; i++)
			{
				for (int j = 0; j < globalStiffnessExpected.NumColumns; j++)
				{
					Assert.Equal(globalStiffnessExpected[i, j], globalStiffnessComputed[i, j],10);
				}
			}
		}
		[Fact]
		private void RefillDisplacements()
		{
			var geometry = new GeometryProperties(40, 40, 1, 2, 2);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var model = new Model(material, geometry);
			model.MakeMesh();
			model.EnumerateDegreesOfFreedom();
			var nodalLoad = new NodalLoad(geometry, EndLoad.BottomEnd);
			Vector rhs = nodalLoad.CalcRHS();
			ISolver solver = new SkylineLdlSolver();
			var femAnalysis = new FEMAnalysis(model, solver, rhs);
			Vector solution = Vector.CreateWithValue(12, 10);
			femAnalysis.RefillDisplacements(solution);
			var displacementsComputed = femAnalysis.displacements;
			var displacementsExpected = Vector.CreateFromArray(new double[] { 0, 0, 0, 0, 0, 0, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 });
			Assert.Equal(displacementsExpected.Length, displacementsComputed.Length);
			for (int i = 0; i < displacementsExpected.Length; i++)
			{
				Assert.Equal(displacementsExpected[i], displacementsComputed[i]);
			}
		}
		[Fact]
		public void SolveTest()
		{
			var geometry = new GeometryProperties(40, 20, 1, 40, 20);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var model = new Model(material, geometry);
			model.MakeMesh();
			model.EnumerateDegreesOfFreedom();
			var nodalLoad = new NodalLoad(geometry, EndLoad.BottomEnd);
			Vector rhs = nodalLoad.CalcRHS();
			ISolver solver = new SkylineLdlSolver();
			var femAnalysis = new FEMAnalysis(model, solver, rhs);
			femAnalysis.Solve();
			var displacementsComputed = femAnalysis.displacements;
			var reader = new Array1DReader(false);
			string inputFile = @"C:\Users\ebank\source\repos\MSolveOne\tests\MGroup.XFEM.IsoXFEM.Tests\Resources\OOSBottomEnd_40x20_it_0_displacements.txt";
			var uexp = reader.ReadFile(inputFile);
			Vector displacementsExpected = Vector.CreateFromArray(uexp, true);
			for (int i = 0; i < displacementsExpected.Length; i++)
			{
				Assert.Equal(displacementsExpected[i], displacementsComputed[i]);
			}
		}
	}
}
