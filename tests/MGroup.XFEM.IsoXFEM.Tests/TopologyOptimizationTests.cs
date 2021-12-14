namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Input;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.MSolve.Discretization.Loads;
	using MGroup.XFEM.IsoXFEM.Solvers;
	using MGroup.XFEM.Materials.Duplicates;

	using Xunit;

	public class TopologyOptimizationTests
	{
		public enum EndLoad
		{
			UpperEnd,
			MiddleEnd,
			BottomEnd
		}
		private static EndLoad endload;
		[Fact]
		private void SolidAreaTest()
		{
			var geometry = new GeometryProperties(2, 1, 1, 2, 1);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var meshGeneration = new MeshGeneration(material, geometry);
			var mesh = meshGeneration.MakeMesh();
			int dimension = 2;
			var xModel = new XModel<IsoXfemElement2D>(dimension);
			foreach (var item in mesh.Item1.Keys)
			{
				xModel.Nodes[item] = mesh.Item1[item];
			}
			foreach (var item in mesh.Item2.Keys)
			{
				xModel.Elements[item] = mesh.Item2[item];
			}
			xModel.Initialize();
			Vector nodalStrainEnergyDensity = Vector.CreateFromArray(new double[] { -50, -50, 100, 100, -200, -200 });
			Vector initialAreas = Vector.CreateWithValue(2, 1);
			Vector areaOfElementsExpected = Vector.CreateFromArray(new double[] { 0.6666666666666666, 0.33333333333333 });
			var areaOfElemenetsComputed = TopologyOptimization.SolidArea(xModel, initialAreas, nodalStrainEnergyDensity);
			for (int i = 0; i < areaOfElementsExpected.Length; i++)
			{
				Assert.Equal(areaOfElementsExpected[i], areaOfElemenetsComputed[i],10);
			}
		}

		[Fact]

		private void UpdatingMLPTest()
		{
			var geometry = new GeometryProperties(40, 20, 1, 40, 20);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var meshGeneration = new MeshGeneration(material, geometry);
			var mesh = meshGeneration.MakeMesh();
			int dimension = 2;
			var xModel = new XModel<IsoXfemElement2D>(dimension);
			foreach (var item in mesh.Item1.Keys)
			{
				xModel.Nodes[item] = mesh.Item1[item];
			}
			foreach (var item in mesh.Item2.Keys)
			{
				xModel.Elements[item] = mesh.Item2[item];
			}
			xModel.Initialize();
			endload = EndLoad.BottomEnd;
			int nodeIDLoad = (geometry.numberOfElementsX + 1) * (geometry.numberOfElementsY + 1) - ((int)endload * (geometry.numberOfElementsY) / 2) - 1;
			Load load;
			load = new Load()
			{
				Node = xModel.Nodes[nodeIDLoad],
				DOF = StructuralDof.TranslationY,
				Amount = 1
			};
			xModel.NodalLoads.Add(load);
			ISolver solver = new SkylineLdlSolver();
			var femAnalysis = new FEMAnalysis(geometry, xModel, solver/*, rhs*/);
			femAnalysis.Initialize();
			var mlp10Iterations = Vector.CreateFromArray(new double[] { 1.1588748787736138, 0.00013116325274719682, 0.00060439138724182077, 0.0014090901824856217, 0.0022817600413313054, 0.0028861382523827179, 0.0035357835523449534 , 0.0042352927267947432, 0.00490602058830891, 0.0053792332498687539, 0.0058575331187078307 });
			var vfi10Iterations = Vector.CreateFromArray(new double[] { 0.99, 0.9801, 0.97029899999999991, 0.96059601, 0.95099004989999991, 0.94148014940099989 , 0.93206534790698992, 0.92274469442792, 0.91351724748364072, 0.9043820750088043 });
			var vfk=0.00;
			Vector initialAreas = Vector.CreateWithValue(800, 1);
			for (int i = 0; i < 10; i++)
			{
				var reader1 = new Array1DReader(false);
				string inputFile1 = @"C:\Users\ebank\source\repos\MSolveOne\tests\MGroup.XFEM.IsoXFEM.Tests\Resources\NodalStrainEnergy_It_0.txt";
				var b = i;
				string step = b.ToString();
				var inputFileSED = inputFile1.Replace("0", step);
				var SED=reader1.ReadFile(inputFileSED);
			    TopologyOptimization.nodalStrainEnergyIt = Vector.CreateZero(SED.Length);
				for (int j=0; j<SED.Length;j++)
				{
					TopologyOptimization.nodalStrainEnergyIt[j] = SED[j];
				}
				TopologyOptimization.mlp = mlp10Iterations[i];
				double vfi = vfi10Iterations[i];
				var reader2 = new Array1DReader(false);
				string inputFile2 = @"C:\Users\ebank\source\repos\MSolveOne\tests\MGroup.XFEM.IsoXFEM.Tests\Resources\LevelSet_It_0.txt";
				var c = i;
				string step2 = c.ToString();
				var inputFileLS = inputFile2.Replace("0", step2);
				var levelSet = reader2.ReadFile(inputFileLS);
				Vector levelSetExpected = Vector.CreateZero(levelSet.Length);
				for (int j = 0; j < levelSetExpected.Length; j++)
				{
					levelSetExpected[j] = levelSet[j];
				}
				var levelSetComputed=TopologyOptimization.UpdatingMLP(xModel, vfi, vfk, initialAreas, 800);
				for (int j = 0; j < levelSetExpected.Length; j++)
				{
					Assert.Equal(levelSetExpected[j], levelSetComputed[j]);
				}
			}
		}

		[Fact]

		public void IsoXfemTest()
		{
			var geometry = new GeometryProperties(40, 20, 1, 40, 20);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var meshGeneration = new MeshGeneration(material, geometry);
			var mesh = meshGeneration.MakeMesh();
			int dimension = 2;
			var xModel = new XModel<IsoXfemElement2D>(dimension);
			foreach (var item in mesh.Item1.Keys)
			{
				xModel.Nodes[item] = mesh.Item1[item];
			}
			foreach (var item in mesh.Item2.Keys)
			{
				xModel.Elements[item] = mesh.Item2[item];
			}
			xModel.Initialize();
			endload = EndLoad.BottomEnd;
			int nodeIDLoad = (geometry.numberOfElementsX + 1) * (geometry.numberOfElementsY + 1) - ((int)endload * (geometry.numberOfElementsY) / 2) - 1;
			Load load;
			load = new Load()
			{
				Node = xModel.Nodes[nodeIDLoad],
				DOF = StructuralDof.TranslationY,
				Amount = 1
			};
			xModel.NodalLoads.Add(load);
			ISolver solver = new SkylineLdlSolver();
			var femAnalysis = new FEMAnalysis(geometry, xModel, solver/*, rhs*/);
			femAnalysis.Initialize();
			TopologyOptimization.IsoXfem(xModel, femAnalysis);
			var resultsComputed = TopologyOptimization.results;
			var reader = new FullMatrixReader(true);
			string inputFile = @"C:\Users\ebank\source\repos\MSolveOne\tests\MGroup.XFEM.IsoXFEM.Tests\Resources\OOSBottomEnd_40x20_SkylineLDL_InitialStiffness_ComputeOnlyOneTime_CorrectMatlabErrors.txt";
			var resultsExpected = reader.ReadFile(inputFile);
			for (int i = 0; i < resultsExpected.NumRows; i++)
			{
				for (int j = 0; j < resultsExpected.NumColumns; j++)
				{
					Assert.Equal(resultsExpected[i,j], resultsComputed[i,j]);
				}
			}
		}

	}
}
