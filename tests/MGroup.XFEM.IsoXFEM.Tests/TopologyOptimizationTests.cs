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
	using MGroup.XFEM.Materials.Duplicates;
	using MGroup.XFEM.IsoXFEM.SolidRatioComputations;
	using Xunit;
	using MGroup.Solvers.Direct;
	using MGroup.Constitutive.Structural;
	using MGroup.NumericalAnalyzers;
	using MGroup.XFEM.Entities;
	using MGroup.XFEM.IsoXFEM.IsoXfemElements;
	using MGroup.XFEM.IsoXFEM.MeshGeneration;

	using System.Linq;

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
		//                      
		//     .________________________________.#0
		//     |                                |
		//     |                                |
		//     |                                |
		//     |                                |
		//     |                                |#1
		//     |                                |      
		//     |                                |
		//     |                                |
		//     |                                |
		//     .________________________________.#2
		//                    
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
			/// <summary>
			/// Define material properties and geometry.
			/// </summary>
			var geometry = new GeometryProperties(40, 20, 1, new int[] { 40, 20});
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			/// <summary>
			/// Create mesh.
			/// </summary>
			IMeshGeneration meshGeneration = new MeshGeneration2D(material, geometry);
			var (nodes, elements) = meshGeneration.MakeMesh();
			/// <summary>
			/// Add Constraints, Using enum Constrained Side.
			/// </summary>
			constrainedSide = ConstrainedSide.Leftside;
			foreach (var node in nodes.Values)
			{
				switch (constrainedSide)
				{
					case ConstrainedSide.Bottomside:
						if (node.Y == 0)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Rightside:
						if (node.X == geometry.length)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Upperside:
						if (node.Y == geometry.height)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Leftside:
						if (node.X == 0)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					default:
						break;
				}
			}
			/// <summary>
			/// X-Model creation.2-D Model.
			/// </summary>
			int dimension = 2;
			var xModel = new IsoXFEM.XModel<IIsoXfemElement>(dimension);
			/// <summary>
			/// Add Subdomain, Nodes and Elements to Model.
			/// </summary>
			xModel.Subdomains[0] = new XSubdomain<IIsoXfemElement>(0);
			foreach (var node in nodes.Keys)
			{
				xModel.Nodes[node] = nodes[node];
			}
			foreach (var element in elements.Keys)
			{
				xModel.Elements[element] = elements[element];
				xModel.Subdomains[0].Elements.Add(elements[element]);
			}
			/// <summary>
			/// Add Loads. Using enum EndLoad in order to choose the node we want to apply the force.
			/// </summary>
			endload = EndLoad.BottomEnd;
			int nodeIDLoad = (geometry.NumberOfElementsX + 1) * (geometry.NumberOfElementsY + 1) - ((int)endload * (geometry.NumberOfElementsY) / 2) - 1;
			Load load;
			load = new Load()
			{
				Node = xModel.Nodes[nodeIDLoad],
				DOF = StructuralDof.TranslationY,
				Amount = 1
			};
			xModel.NodalLoads.Add(load);
			/// <summary>
			/// Initialize the Model.
			/// </summary>
			xModel.Initialize();
			/// <summary>
			/// Defines Skyline Solver.
			/// </summary>
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(xModel);
			var solver = solverFactory.BuildSolver(algebraicModel);
			/// Defines solidRatio. The Problem is 2D so SolidArea is selected.
			/// </summary>
			ISolidRatio solidRatio = new SolidArea(xModel, Vector.CreateWithValue(xModel.Elements.Count, xModel.Elements.First().Value.SizeOfElement));
			/// <summary>
			/// Defines the topology Optimization.
			/// </summary>
			var topologyOptimization = new TopologyOptimization(xModel, solidRatio, solver, algebraicModel);
			/// <summary>
			/// Defines expected results for the firsts 10 iterations.
			/// </summary>
			var mlp10Iterations = Vector.CreateFromArray(new double[] { 1.1588748787736138, 0.00013116325274719682, 0.00060439138724182077, 0.0014090901824856217, 0.0022817600413313054, 0.0028861382523827179, 0.0035357835523449534, 0.0042352927267947432, 0.00490602058830891, 0.0053792332498687539, 0.0058575331187078307 });
			var vfi10Iterations = Vector.CreateFromArray(new double[] { 0.99, 0.9801, 0.97029899999999991, 0.96059601, 0.95099004989999991, 0.94148014940099989, 0.93206534790698992, 0.92274469442792, 0.91351724748364072, 0.9043820750088043 });
			var vfk = 0.00;
			/// <summary>
			/// Check results.Using txt files from Resources.
			/// </summary>
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
				var levelSetComputed = topologyOptimization.UpdatingMLP(vfi, vfk, 800);
				for (int j = 0; j < levelSetExpected.Length; j++)
				{
					Assert.Equal(levelSetExpected[j], levelSetComputed[j]);
				}
			}
		}

		[Fact]

		public void IsoXfemTest()
		{
			/// <summary>
			/// Define material properties and geometry.
			/// </summary>
			var geometry = new GeometryProperties(40, 20, 1, new int[] { 40, 20 });
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			/// <summary>
			/// Create mesh.
			/// </summary>
			IMeshGeneration meshGeneration = new MeshGeneration2D(material, geometry);
			var (nodes, elements) = meshGeneration.MakeMesh();
			/// <summary>
			/// Add Constraints, Using enum Constrained Side.
			/// </summary>
			constrainedSide = ConstrainedSide.Leftside;
			foreach (var node in nodes.Values)
			{
				switch (constrainedSide)
				{
					case ConstrainedSide.Bottomside:
						if (node.Y == 0)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Rightside:
						if (node.X == geometry.length)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Upperside:
						if (node.Y == geometry.height)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					case ConstrainedSide.Leftside:
						if (node.X == 0)
						{
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
							node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
						}
						break;
					default:
						break;
				}
			}
			/// <summary>
			/// X-Model creation.2-D Model.
			/// </summary>
			int dimension = 2;
			var xModel = new IsoXFEM.XModel<IIsoXfemElement>(dimension);
			/// <summary>
			/// Add Subdomain, Nodes and Elements to Model.
			/// </summary>
			xModel.Subdomains[0] = new XSubdomain<IIsoXfemElement>(0);
			foreach (var node in nodes.Keys)
			{
				xModel.Nodes[node] = nodes[node];
			}
			foreach (var element in elements.Keys)
			{
				xModel.Elements[element] = elements[element];
				xModel.Subdomains[0].Elements.Add(elements[element]);
			}
			/// <summary>
			/// Add Loads. Using enum EndLoad in order to choose the node we want to apply the force.
			/// </summary>
			endload = EndLoad.BottomEnd;
			int nodeIDLoad = (geometry.NumberOfElementsX + 1) * (geometry.NumberOfElementsY + 1) - ((int)endload * (geometry.NumberOfElementsY) / 2) - 1;
			Load load;
			load = new Load()
			{
				Node = xModel.Nodes[nodeIDLoad],
				DOF = StructuralDof.TranslationY,
				Amount = 1
			};
			xModel.NodalLoads.Add(load);
			/// <summary>
			/// Initialize the Model.
			/// </summary>
			xModel.Initialize();
			/// <summary>
			/// Defines Skyline Solver.
			/// </summary>
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(xModel);
			var solver = solverFactory.BuildSolver(algebraicModel);
			/// <summary>
			/// Defines solidRatio. The Problem is 2D so SolidArea is selected.
			/// </summary>
			ISolidRatio solidRatio = new SolidArea(xModel, Vector.CreateWithValue(xModel.Elements.Count, xModel.Elements.First().Value.SizeOfElement));
			/// <summary>
			/// Defines the topology Optimization and Optimize the problem with IsoXfem Method.
			/// </summary>
			var topologyOptimization = new TopologyOptimization(xModel, solidRatio, solver, algebraicModel);
			topologyOptimization.IsoXfem();
			var resultsComputed = topologyOptimization.results;
			/// <summary>
			/// Expected Results from txt file.
			/// </summary>
			var reader = new FullMatrixReader(true);
			string inputFile = @"C:\Users\ebank\source\repos\MSolveOne\tests\MGroup.XFEM.IsoXFEM.Tests\Resources\OOS_BottomEnd_40x20_Using_ConformingSubcells.txt";
			var resultsExpected = reader.ReadFile(inputFile);
			/// <summary>
			/// Check Results.
			/// </summary>
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
