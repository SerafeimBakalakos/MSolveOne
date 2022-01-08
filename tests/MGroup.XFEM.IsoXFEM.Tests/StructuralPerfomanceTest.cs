namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.Constitutive.Structural;
	using MGroup.LinearAlgebra.Distributed;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.NumericalAnalyzers;
	using MGroup.Solvers.Direct;
	using MGroup.XFEM.Entities;
	using MGroup.XFEM.Materials.Duplicates;
	using MGroup.Solvers.LinearSystem;
	using Xunit;
	using MGroup.MSolve.Discretization.Loads;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.MSolve.Discretization;
	using MGroup.LinearAlgebra.Reduction;

	public class StructuralPerfomanceTest
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
		private static ConstrainedSide constrainedSide;

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
		public void ComputeStrainEnergyandStrainEnergyDensity()
		{
			var geometry = new GeometryProperties(20, 20, 1, 2, 2);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			/// <summary>
			/// Create mesh.
			/// </summary>
			var meshGeneration = new MeshGeneration(material, geometry);
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
			int nodeIDLoad = (geometry.numberOfElementsX + 1) * (geometry.numberOfElementsY + 1) - ((int)endload * (geometry.numberOfElementsY) / 2) - 1;
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
			algebraicModel.OrderDofs();
			var solver = solverFactory.BuildSolver(algebraicModel);

			//GlobalVector globalDisplacements =algebraicModel.CreateZeroVector();
			//globalDisplacements.SingleVector= Vector.CreateFromArray(new double[] { 1.76924266492499, 2.43273081966077, 0.174565886551294, 2.18549731446979, -2.05458622432392, 2.22010344064856, 2.24365209361063, 4.97664957363316, 0.162098204124805, 5.50209505074586, -3.20059751636863, 7.51106771335219 });
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
			/// <summary>
			/// Construct structural perfomance.
			/// </summary>
			var solution = solver.LinearSystem.Solution;
			double initialArea = 100;
			var structuralPerfomance = new StructuralPerfomance(xModel.Nodes, xModel.Elements, initialArea, algebraicModel);
			/// <summary>
			/// Run Method ComputeStrainEnergyandStrainEnergyDensity.
			/// </summary>
			var (strainEnergyComputed, strainEnergyDensityComputed) = structuralPerfomance.ComputeStrainEnergyandStrainEnergyDensity(solution);
			/// <summary>
			/// Define expected results.
			/// </summary>
			Vector strainEnergyExpected = Vector.CreateFromArray(new double[] { 1.015275479392279, 1.088279753481190, 1.320505235676104, 0.331473388126517 });
			Vector strainEnergyDensityExpected = Vector.CreateFromArray(new double[] { 0.010152754793923, 0.010882797534812, 0.013205052356761, 0.003314733881265 });
			/// <summary>
			/// Check results.
			/// </summary>
			for (int i = 0; i < strainEnergyExpected.Length; i++)
			{
				Assert.Equal(strainEnergyExpected[i], strainEnergyComputed[i], 8);
				Assert.Equal(strainEnergyDensityExpected[i], strainEnergyDensityComputed[i], 8);
			}
		}
		[Fact]
		public void ComputeNodalStrainEnergyDensityTest()
		{
			var geometry = new GeometryProperties(20, 20, 1, 2, 2);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			/// <summary>
			/// Create mesh.
			/// </summary>
			var meshGeneration = new MeshGeneration(material, geometry);
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
			int nodeIDLoad = (geometry.numberOfElementsX + 1) * (geometry.numberOfElementsY + 1) - ((int)endload * (geometry.numberOfElementsY) / 2) - 1;
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
			algebraicModel.OrderDofs();
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
			/// <summary>
			/// Construct structural perfomance.
			/// </summary>
			var solution = solver.LinearSystem.Solution;
			double initialArea = 100;
			var structuralPerfomance = new StructuralPerfomance(xModel.Nodes, xModel.Elements, initialArea, algebraicModel);
			/// <summary>
			/// Run Method ComputeStrainEnergyAndNodalSEDensity.
			/// </summary>
			var (strainEnergyComputed, nodalStrainEnerdyDensityComputed) = structuralPerfomance.ComputeStrainEnergyAndNodalSEDensity(solution);
			var strainEnergyTotalComputed = strainEnergyComputed.Sum();
			/// <summary>
			/// Define expected results.
			/// </summary>
			double strainEnergyTotalExpected = 3.755533856676090;
			Vector nosalStrainEnergyDensityExpected = Vector.CreateFromArray(new double[] { 0.010152754793923, 0.010517776164367, 0.010882797534812,
			0.011678903575342, 0.009388834641690,0.007098765708039,0.013205052356761, 0.008259893119013,0.003314733881265});
			/// <summary>
			/// Check results.
			/// </summary>
			Assert.Equal(nosalStrainEnergyDensityExpected.Length, nodalStrainEnerdyDensityComputed.Length);
			Assert.Equal(strainEnergyTotalExpected, strainEnergyTotalComputed, 10);
			for (int i = 0; i < nosalStrainEnergyDensityExpected.Length; i++)
			{
				Assert.Equal(nosalStrainEnergyDensityExpected[i], nodalStrainEnerdyDensityComputed[i], 10);
			}
		}
	}
}
