using MGroup.XFEM.IsoXFEM.Solvers;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;
using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Entities;
using MGroup.Solvers.Direct;
using MGroup.Constitutive.Structural;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.NumericalAnalyzers;

namespace MGroup.XFEM.IsoXFEM.Tests
{

	public class Example1
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
		public enum EndLoad
		{
			UpperEnd,
			MiddleEnd,
			BottomEnd
		}
		private static EndLoad endload;

		public static void RunExample1()
        {
            var geometry = new GeometryProperties(40, 20, 1, 40, 20);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;
			constrainedSide = ConstrainedSide.Leftside;
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
			var xModel = new XModel<IsoXfemElement2D>(dimension);
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
			int nodeIDLoad =  (geometry.numberOfElementsX + 1) * (geometry.numberOfElementsY + 1) - ((int)endload * (geometry.numberOfElementsY)/2) - 1;
			Load load;
			load = new Load()
			{
				Node = xModel.Nodes[nodeIDLoad],
				DOF = StructuralDof.TranslationY,
				Amount = 1
			};
			xModel.NodalLoads.Add(load);
			xModel.Initialize();
			//ISolver solver0 = new SkylineLdlSolver();
			//var femAnalysis = new FEMAnalysis(geometry, xModel, solver0/*, rhs*/);
			//femAnalysis.Initialize();
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
			//var solutionof1stElement=algebraicModel.ExtractElementVector(solution, xModel.Elements[0]);
			//var topologyOptimization = new TopologyOptimization(xModel, femAnalysis);
   //         topologyOptimization.IsoXfem();
        }
    }
}
