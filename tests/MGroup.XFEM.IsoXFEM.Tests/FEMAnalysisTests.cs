namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System.IO;

	using MGroup.LinearAlgebra.Input;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.MSolve.Discretization.Loads;
	using MGroup.XFEM.IsoXFEM.Solvers;
	using MGroup.XFEM.Materials.Duplicates;
	using MGroup.NumericalAnalyzers;
	using MGroup.Solvers.Direct;

	using Xunit;
	using MGroup.Constitutive.Structural;
	using MGroup.XFEM.Entities;

	public class FEMAnalysisTests
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
		private  EndLoad endload;

		[Fact]
		private void AssembleStiffnessMatrix()
		{
			var geometry = new GeometryProperties(30, 10, 1, 3, 1);
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
			foreach (var item in mesh.Item1.Keys)
			{
				xModel.Nodes[item] = mesh.Item1[item];
			}
			foreach (var item in mesh.Item2.Keys)
			{
				xModel.Elements[item] = mesh.Item2[item];
			}
			endload = EndLoad.MiddleEnd;
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
			ISolver solver = new SkylineLdlSolver();
			var femAnalysis = new FEMAnalysis(geometry, xModel, solver/*, rhs*/);
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
			foreach (var item in mesh.Item1.Keys)
			{
				xModel.Nodes[item] = mesh.Item1[item];
			}
			foreach (var item in mesh.Item2.Keys)
			{
				xModel.Elements[item] = mesh.Item2[item];
			}
			xModel.Initialize();
			ISolver solver = new SkylineLdlSolver();
			var femAnalysis = new FEMAnalysis(geometry, xModel, solver/*, rhs*/);
			femAnalysis.Initialize();
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
			//Use FEMAnalysis
			//ISolver solver = new SkylineLdlSolver();
			//var femAnalysis = new FEMAnalysis(geometry, xModel, solver/*, rhs*/);
			//femAnalysis.Initialize();
			//femAnalysis.Solve();
			//var displacementsComputed = femAnalysis.displacements;
			//Use MSolve.NumericalAnalyzers
			/// <summary>
			/// Defines Skyline Solver.
			/// </summary>
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(xModel);
			var solver = solverFactory.BuildSolver(algebraicModel);

			/// <summary>
			/// Defines Problem type as Structural.
			/// </summary>
			var provider = new ProblemStructural(xModel, algebraicModel, solver);

			/// <summary>
			/// Defines Analyzers.
			/// Chlid Analyzer: Linear
			/// Parent Analyzer: Static
			/// </summary>
			var childAnalyzer = new LinearAnalyzer(xModel, algebraicModel, solver, provider);
			var parentAnalyzer = new StaticAnalyzer(xModel, algebraicModel, solver, provider, childAnalyzer);

			/// <summary>
			/// Run the anlaysis.
			/// </summary>
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();
			var solution = solver.LinearSystem.Solution.SingleVector;

			//Extract Results from txt file
			var reader = new Array1DReader(false);
			string inputFile = @"C:\Users\ebank\source\repos\MSolveOne\tests\MGroup.XFEM.IsoXFEM.Tests\Resources\OOSBottomEnd_40x20_it_0_displacements.txt";
			var uexp = reader.ReadFile(inputFile);
			Vector displacementsExpected = Vector.CreateFromArray(uexp, true);
			for (int i = 0; i < solution.Length; i++)
			{
				Assert.Equal(displacementsExpected[i+42], solution[i],10);
			}
		}
	}
}
