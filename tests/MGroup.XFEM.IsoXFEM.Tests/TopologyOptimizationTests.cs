namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Input;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.MSolve.Discretization.Loads;
	using MGroup.XFEM.IsoXFEM.Solvers;
	using MGroup.XFEM.Materials.Duplicates;
	using MGroup.XFEM.IsoXFEM.SolidRatioComputations;
	using Xunit;
	using MGroup.Solvers.Direct;
	using MGroup.Constitutive.Structural;
	using MGroup.NumericalAnalyzers;
	using MGroup.XFEM.Entities;

	public class TopologyOptimizationTests
	{
		//                      #2
		//     .________________________________.
		//     |                                |
		//     |                                |
		//     |                                |
		//     |                                |
		//     |                                |
		//  #3 |                                |      #1
		//     |                                |
		//     |                                |
		//     |                                |
		//     .________________________________.
		//                    #0
		public enum ConstrainedSide
		{
			Bottomside,
			Rightside,
			Upperside,
			Leftside,
		}
		private static ConstrainedSide constrainedSide = ConstrainedSide.Leftside;
		public enum EndLoad
		{
			UpperEnd,
			MiddleEnd,
			BottomEnd
		}
		private static EndLoad endload;
		
		[Fact]

		private void UpdatingMLPTest()
		{
			var geometry = new GeometryProperties(40, 20, 1, 40, 20);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			var meshGeneration = new MeshGeneration(material, geometry);
			var mesh = meshGeneration.MakeMesh();
			foreach (var item in mesh.Item1.Values)
			{
				switch (constrainedSide)
				{
					case ConstrainedSide.Bottomside:
						if (item.Y == 0)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Rightside:
						if (item.X == geometry.length)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Upperside:
						if (item.Y == geometry.height)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Leftside:
						if (item.X == 0)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					default:
						break;
				}
			}
			int dimension = 2;
			var xModel = new IsoXFEM.XModel<IsoXfemElement2D>(dimension);
			xModel.Subdomains[0] = new XSubdomain<IsoXfemElement2D>(0);
			foreach (var item in mesh.Item1.Keys)
			{
				xModel.Nodes[item] = mesh.Item1[item];
			}
			foreach (var item in mesh.Item2.Keys)
			{
				xModel.Elements[item] = mesh.Item2[item];
				xModel.Subdomains[0].Elements.Add(mesh.Item2[item]);
			}
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
			xModel.Initialize();
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(xModel);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var provider = new ProblemStructural(xModel, algebraicModel, solver);
			var childAnalyzer = new LinearAnalyzer(xModel, algebraicModel, solver, provider);
			var parentAnalyzer = new StaticAnalyzer(xModel, algebraicModel, solver, provider, childAnalyzer);
			var mlp10Iterations = Vector.CreateFromArray(new double[] { 1.1588748787736138, 0.00013116325274719682, 0.00060439138724182077, 0.0014090901824856217, 0.0022817600413313054, 0.0028861382523827179, 0.0035357835523449534, 0.0042352927267947432, 0.00490602058830891, 0.0053792332498687539, 0.0058575331187078307 });
			var vfi10Iterations = Vector.CreateFromArray(new double[] { 0.99, 0.9801, 0.97029899999999991, 0.96059601, 0.95099004989999991, 0.94148014940099989, 0.93206534790698992, 0.92274469442792, 0.91351724748364072, 0.9043820750088043 });
			var vfk = 0.00;
			Vector initialAreas = Vector.CreateWithValue(800, 1);
			var topologyOptimization = new TopologyOptimization(xModel, parentAnalyzer, solver, algebraicModel);
			for (int i = 0; i < 10; i++)
			{
				var reader1 = new Array1DReader(false);
				string inputFile1 = @"C:\Users\ebank\source\repos\MSolveOne\tests\MGroup.XFEM.IsoXFEM.Tests\Resources\NodalStrainEnergy_It_0.txt";
				var b = i;
				string step = b.ToString();
				var inputFileSED = inputFile1.Replace("0", step);
				var SED = reader1.ReadFile(inputFileSED);
				topologyOptimization.nodalStrainEnergyIt = Vector.CreateZero(SED.Length);
				for (int j = 0; j < SED.Length; j++)
				{
					topologyOptimization.nodalStrainEnergyIt[j] = SED[j];
				}
				topologyOptimization.mlp = mlp10Iterations[i];
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
				var levelSetComputed = topologyOptimization.UpdatingMLP(vfi, vfk, initialAreas, 800);
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
			foreach (var item in mesh.Item1.Values)
			{
				switch (constrainedSide)
				{
					case ConstrainedSide.Bottomside:
						if (item.Y == 0)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Rightside:
						if (item.X == geometry.length)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Upperside:
						if (item.Y == geometry.height)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Leftside:
						if (item.X == 0)
						{
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							item.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					default:
						break;
				}
			}
			int dimension = 2;
			var xModel = new IsoXFEM.XModel<IsoXfemElement2D>(dimension);
			xModel.Subdomains[0] = new XSubdomain<IsoXfemElement2D>(0);
			foreach (var item in mesh.Item1.Keys)
			{
				xModel.Nodes[item] = mesh.Item1[item];
			}
			foreach (var item in mesh.Item2.Keys)
			{
				xModel.Elements[item] = mesh.Item2[item];
				xModel.Subdomains[0].Elements.Add(mesh.Item2[item]);
			}
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
			xModel.Initialize();
			//ISolver solver = new SkylineLdlSolver();
			//var femAnalysis = new FEMAnalysis(geometry, xModel, solver/*, rhs*/);
			//femAnalysis.Initialize();
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(xModel);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var provider = new ProblemStructural(xModel, algebraicModel, solver);
			var childAnalyzer = new LinearAnalyzer(xModel, algebraicModel, solver, provider);
			var parentAnalyzer = new StaticAnalyzer(xModel, algebraicModel, solver, provider, childAnalyzer);
			var topologyOptimization = new TopologyOptimization(xModel, parentAnalyzer, solver, algebraicModel);
			topologyOptimization.IsoXfem();
			var resultsComputed = topologyOptimization.results;
			var reader = new FullMatrixReader(true);
			string inputFile = @"C:\Users\ebank\source\repos\MSolveOne\tests\MGroup.XFEM.IsoXFEM.Tests\Resources\OOS_BottomEnd_40x20_InitialStiffness_ComputeOnlyOneTime_CorrectMatlabErrors.txt";
			var resultsExpected = reader.ReadFile(inputFile);
			for (int i = 0; i < resultsExpected.NumRows; i++)
			{
				for (int j = 0; j < resultsExpected.NumColumns; j++)
				{
					Assert.Equal(resultsExpected[i, j], resultsComputed[i, j]);
				}
			}
		}

	}
}
