using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.ContinuumElements;
using MGroup.Environments;
using MGroup.FEM.Entities;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.PFetiDP;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.DDM.Tests.Commons;

namespace MGroup.Solvers.DDM.Tests.PFetiDP
{
	public class UniformExample
	{
		public static void Run()
		{
			(Model model, ComputeNodeTopology nodeTopology) = DescribeModel().BuildMultiSubdomainModel();
			(ISolver solver, IAlgebraicModel algebraicModel) = SetupSolver(model, nodeTopology);

			// Linear static analysis
			var problem = new ProblemStructural(model, algebraicModel, solver);
			var childAnalyzer = new LinearAnalyzer(model, algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, childAnalyzer);
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			Console.WriteLine($"Num dofs = {solver.LinearSystem.Solution.Length()}");
		}

		private static UniformDdmModelBuilder3D DescribeModel()
		{
			var builder = new UniformDdmModelBuilder3D();
			builder.MinCoords = new double[] { 0, 0, 0 };
			builder.MaxCoords = new double[] { 8, 4, 4 }; 
			builder.NumElementsTotal = new int[] { 110, 55, 55 };
			builder.NumSubdomains = new int[] { 22, 11, 11};
			builder.NumClusters = new int[] { 1, 1, 1};
			builder.MaterialHomogeneous = new ElasticMaterial3D() { YoungModulus = 2E7, PoissonRatio = 0.3 };
			builder.PrescribeDisplacement(UniformDdmModelBuilder3D.BoundaryRegion.MinX, StructuralDof.TranslationX, 0.0);
			builder.PrescribeDisplacement(UniformDdmModelBuilder3D.BoundaryRegion.MinX, StructuralDof.TranslationY, 0.0);
			builder.PrescribeDisplacement(UniformDdmModelBuilder3D.BoundaryRegion.MinX, StructuralDof.TranslationZ, 0.0);
			builder.DistributeLoadAtNodes(UniformDdmModelBuilder3D.BoundaryRegion.MaxX, StructuralDof.TranslationY, 10000.0);

			return builder;
		}

		private static (ISolver solver, IAlgebraicModel algebraicModel) SetupSolver(Model model, ComputeNodeTopology nodeTopology)
		{
			// Environment
			IComputeEnvironment environment = new TplSharedEnvironment(false);
			environment.Initialize(nodeTopology);

			// Corner dofs
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			ICornerDofSelection cornerDofs = UniformDdmModelBuilder3D.FindCornerDofs(model);

			// Solver settings
			var psmMatrices = new PsmSubdomainMatrixManagerSymmetricSuiteSparse.Factory();
			var fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricSuiteSparse.Factory(true);
			var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricSuiteSparse();

			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, psmMatrices, cornerDofs, fetiDPMatrices);
			solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
			solverFactory.EnableLogging = true;
			solverFactory.ExplicitSubdomainMatrices = false;
			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 100,
				ResidualTolerance = 1E-7,
				UseObjectiveConvergenceCriterion = false
			};

			// Create solver
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(model, algebraicModel);
			return (solver, algebraicModel);
		}
	}
}
