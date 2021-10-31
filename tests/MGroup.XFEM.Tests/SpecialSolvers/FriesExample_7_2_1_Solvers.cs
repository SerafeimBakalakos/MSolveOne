using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.DDM;
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.Output;
using MGroup.Solvers.DDM.PFetiDP;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.Direct;
using MGroup.XFEM.Analysis;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Boundaries;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Solvers.PFetiDP;
using Xunit;

namespace MGroup.XFEM.Tests.SpecialSolvers.HybridFries
{
	public static class FriesExample_7_2_1
	{
		private enum SolverChoice 
		{ 
			DirectManaged, DirectNative, PfetiDPManaged, PfetiDPNative 
		}

		private static string outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Example1\";
		private static string outputPlotDirectory = outputDirectory + "plots";
		private const bool enablePlotting = false;

		private const int numElementsMin = 25;
		private static readonly int[] numElements = new int[] { 9 * numElementsMin, 2 * numElementsMin, numElementsMin };
		private const int numSubdomainsMin = 5;
		private static readonly int[] numSubdomains = new int[] { 9 * numSubdomainsMin, 2 * numSubdomainsMin, numSubdomainsMin };

		private const int maxIterations = 11;
		private const double fractureToughness = double.MaxValue;

		private const bool reanalysis = false;
		private const double psmTolerance = 1E-10;

		[Fact]
		public static void RunExampleWithDirectSolver()
		{
			XModel<IXCrackElement> model = FriesExample_7_2_1_Model.DescribePhysicalModel(numElements).BuildSingleSubdomainModel();
			if (enablePlotting)
			{
				FriesExample_7_2_1_Model.CreateGeometryModel(model, numElements, outputPlotDirectory);
				FriesExample_7_2_1_Model.SetupEnrichmentOutput(model, outputPlotDirectory);
			}
			else
			{
				FriesExample_7_2_1_Model.CreateGeometryModel(model, numElements);
			}
			SolverChoice solverChoice = SolverChoice.DirectNative;
			(ISolver solver, IAlgebraicModel algebraicModel) = SetupDirectSolver(model, solverChoice);
			RunAnalysis(model, algebraicModel, solver, solverChoice);
		}

		[Fact]
		public static void RunExampleWithPFetiDPSolver()
		{
			int[] numClusters = { 1, 1, 1 };
			(XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
					= FriesExample_7_2_1_Model.DescribePhysicalModel(numElements, numSubdomains, numClusters)
					.BuildMultiSubdomainModel();
			if (enablePlotting)
			{
				FriesExample_7_2_1_Model.CreateGeometryModel(model, numElements, outputPlotDirectory);
				FriesExample_7_2_1_Model.SetupEnrichmentOutput(model, outputPlotDirectory);
			}
			else
			{
				FriesExample_7_2_1_Model.CreateGeometryModel(model, numElements);
			}

			//SolverChoice solverChoice = SolverChoice.PfetiDPManaged;
			SolverChoice solverChoice = SolverChoice.PfetiDPNative;
			(ISolver solver, IAlgebraicModel algebraicModel, DdmLogger logger) 
				= SetupPFetiDPSolver(model, nodeTopology, solverChoice, numSubdomains);
			RunAnalysis(model, algebraicModel, solver, solverChoice);

			string path = Path.Combine(outputDirectory, "pfetidp_convergence.txt");
			logger.WriteToFile(path, true);
		}

		private static void RunAnalysis(XModel<IXCrackElement> model, IAlgebraicModel algebraicModel, ISolver solver, 
			SolverChoice solverChoice)
		{
			var domainBoundary = new RectangularDomainBoundary(
				FriesExample_7_2_1_Model.minCoords, FriesExample_7_2_1_Model.maxCoords);
			var termination = new TerminationLogic.Or(
				new FractureToughnessTermination(fractureToughness),
				new CrackExitsDomainTermination(domainBoundary));
			var analyzer = new QuasiStaticLefmAnalyzer(model, algebraicModel, solver, maxIterations, termination);

			var msg = new StringBuilder();
			msg.Append($"{DateTime.Now}, solver={solverChoice.ToString()}");
			msg.AppendLine($", numElements={numElements[0]}x{numElements[1]}x{numElements[2]}");
			if (solverChoice == SolverChoice.PfetiDPManaged || solverChoice == SolverChoice.PfetiDPNative)
			{
				msg.Append($"numSubdomains={numSubdomains[0]}x{numSubdomains[1]}x{numSubdomains[2]}");
				msg.AppendLine($", reanalysis={reanalysis}, PSM tolerance={psmTolerance}");
			}

			var normLogger = new SolutionNormLogger(Path.Combine(outputDirectory, "solution_norm.txt"));
			normLogger.ExtraInfo = msg.ToString();
			analyzer.Results.Add(normLogger);
			analyzer.Logger.ExtraInfo = msg.ToString();
			//analyzer.Results.Add(new StructuralFieldWriter(model, outputDirectory));

			analyzer.Analyze();
			analyzer.Logger.WriteToFile(Path.Combine(outputDirectory, "performance.txt"), true);
		}

		private static (ISolver, IAlgebraicModel) SetupDirectSolver(XModel<IXCrackElement> model, SolverChoice solverChoice)
		{
			if (solverChoice == SolverChoice.DirectManaged)
			{
				//TODO: Use CSparse.NET here
				var solverFactory = new SkylineSolver.Factory();
				var algebraicModel = solverFactory.BuildAlgebraicModel(model);
				var solver = solverFactory.BuildSolver(algebraicModel);
				return (solver, algebraicModel);
			}
			else if (solverChoice == SolverChoice.DirectNative)
			{
				var solverFactory = new SuiteSparseSolver.Factory();
				var algebraicModel = solverFactory.BuildAlgebraicModel(model);
				var solver = solverFactory.BuildSolver(algebraicModel);
				return (solver, algebraicModel);
			}
			else
			{
				throw new NotImplementedException();
			}
		}

		private static (ISolver, IAlgebraicModel, DdmLogger) SetupPFetiDPSolver(XModel<IXCrackElement> model, 
			ComputeNodeTopology nodeTopology, SolverChoice solverChoice, int[] numSubdomains)
		{
			// Environment
			IComputeEnvironment environment = new SequentialSharedEnvironment();
			environment.Initialize(nodeTopology);

			// Corner dofs
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			IDofType[] stdDofs = { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ };

			int minMultiplicity = 3;
			if (numSubdomains[2] == 1)
			{
				minMultiplicity = 2;
			}
			var cornerDofs = new CrackFetiDPCornerDofsPlusLogging(environment, model, stdDofs,
				sub => UniformDdmCrackModelBuilder3D.FindCornerNodes(sub, minMultiplicity), 0);
			if (numSubdomains[2] == 1)
			{
				//// We need an extra corner node at the edge subdomains
				//int lastNodeID = model.Nodes.Count - 1;
				//double[] minCoords = FriesExample_7_2_1_Model.minCoords;
				//double[] maxCoords = FriesExample_7_2_1_Model.maxCoords;
				//double[] target = { maxCoords[0], 0.5 * (minCoords[1] + maxCoords[1]), minCoords[2] };
				//double tol = 1E-4;
				//XNode extraNode = model.Nodes.Values.Where(
				//	n => (Math.Abs(n.X - target[0]) <= tol) && (Math.Abs(n.Y - target[1]) <= tol) && (Math.Abs(n.Z - target[2]) <= tol))
				//	.Single();

				//cornerDofs.AddStdCornerNode(8, extraNode.ID);
				//cornerDofs.AddStdCornerNode(17, extraNode.ID);
			}

			if (enablePlotting)
			{
				model.ModelObservers.Add(new PartitioningPlotter(outputPlotDirectory, model, 3));
				model.ModelObservers.Add(new CornerNodesPlotter(environment, model, cornerDofs, outputPlotDirectory));
			}

			// Solver settings
			IPsmSubdomainMatrixManagerFactory<SymmetricCscMatrix> psmMatrices;
			IFetiDPSubdomainMatrixManagerFactory<SymmetricCscMatrix> fetiDPMatrices;
			IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix;
			if (solverChoice == SolverChoice.PfetiDPManaged)
			{
				psmMatrices = new PsmSubdomainMatrixManagerSymmetricCSparse.Factory();
				fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricCSparse.Factory();
				coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCSparse();
			}
			else if (solverChoice == SolverChoice.PfetiDPNative)
			{
				psmMatrices = new PsmSubdomainMatrixManagerSymmetricSuiteSparse.Factory();
				fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricSuiteSparse.Factory();
				coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricSuiteSparse();
			}
			else
			{
				throw new NotImplementedException();
			}

			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, psmMatrices, cornerDofs, fetiDPMatrices);
			solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
			solverFactory.EnableLogging = true;
			solverFactory.ExplicitSubdomainMatrices = false;
			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = psmTolerance
			};

			if (reanalysis)
			{
				var observer = new SubdomainEnrichmentsModifiedObserver();
				model.GeometryModel.Enricher.Observers.Add(observer);
				var reanalysisOptions = PFetiDPReanalysisOptions.CreateWithAllEnabled(observer);
				//var reanalysisOptions = PFetiDPReanalysisOptions.CreateWithAllDisabled();
				reanalysisOptions.PreviousSolution = false; // This causes errors if enabled

				solverFactory.ReanalysisOptions = reanalysisOptions;
				solverFactory.SubdomainTopology = new SubdomainTopologyOptimized();
				solverFactory.ExplicitSubdomainMatrices = true;
			}

			// Create solver
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(model, algebraicModel);
			return (solver, algebraicModel, solver.LoggerDdm);
		}
	}
}
