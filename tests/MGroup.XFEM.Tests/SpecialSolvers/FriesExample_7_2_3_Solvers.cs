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
using MGroup.Solvers.DDM.FetiDP;
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.InterfaceProblem;
using MGroup.Solvers.DDM.FetiDP.Preconditioning;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.Output;
using MGroup.Solvers.DDM.PFetiDP;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.Direct;
using MGroup.XFEM.Analysis;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Boundaries;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Solvers.PaisReanalysis;
using MGroup.XFEM.Solvers.PFetiDP;
using Xunit;

namespace MGroup.XFEM.Tests.SpecialSolvers.HybridFries
{
	public static class FriesExample_7_2_3_Solvers
	{
		public enum PreconditionerFetiDP
		{
			Dirichlet, DiagonalDirichlet, Lumped
		}

		private enum SolverChoice 
		{ 
			DirectManaged, DirectNative, DirectReanalysis, PfetiDPManaged, PfetiDPNative, FetiDPManaged, FetiDPNative
		}

		public enum ReanalysisExtraDofs
		{
			None, LimitedNearModified, CrackStepNearModified, AllNearModified
		}

		public static string outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Example2\";
		public static string outputPlotDirectory = outputDirectory + "plots";
		public static bool enablePlotting = true;

		public static int numElementsMin = 10;
		public static int[] numElements = new int[] { 2 * numElementsMin, numElementsMin, 2 * numElementsMin };
		public static int numSubdomainsMin = 1;
		public static int[] numSubdomains = new int[] { 2 * numSubdomainsMin, numSubdomainsMin, 2 * numSubdomainsMin };
		public static int[][] numElementsPerSubdomain = null;
		
		public static int maxIterations = 16;
		public const double fractureToughness = double.MaxValue;

		public static bool ddmReanalysis = false;
		public static ReanalysisExtraDofs reanalysisExtraDofs = ReanalysisExtraDofs.AllNearModified;
		public static double iterTol = 1E-10;
		public static bool objectivePcgCriterion = false;
		public static bool multiThreaded = false;

		public const bool explicitPsmMatrices = false;
		public const bool unsafeOptimizations = true;
		public static PreconditionerFetiDP preconditionerFetiDP = PreconditionerFetiDP.Dirichlet;

		[Fact]
		public static void RunExampleWithDirectSolver()
		{
			XModel<IXCrackElement> model = FriesExample_7_2_3_Model.DescribePhysicalModel(numElements).BuildSingleSubdomainModel();
			if (enablePlotting)
			{
				FriesExample_7_2_3_Model.CreateGeometryModel(model, numElements, outputPlotDirectory);
				FriesExample_7_2_3_Model.SetupEnrichmentOutput(model, outputPlotDirectory);
			}
			else
			{
				FriesExample_7_2_3_Model.CreateGeometryModel(model, numElements);
			}
			SolverChoice solverChoice = SolverChoice.DirectNative;
			(ISolver solver, IAlgebraicModel algebraicModel) = SetupDirectSolver(model, solverChoice);
			RunAnalysis(model, algebraicModel, solver, solverChoice);
		}

		[Fact]
		public static void RunExampleWithFetiDPSolver()
		{
			int[] numClusters = { 1, 1, 1 };
			(XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
					= FriesExample_7_2_3_Model.DescribePhysicalModel(numElements, numSubdomains, numClusters)
					.BuildMultiSubdomainModel();
			if (enablePlotting)
			{
				FriesExample_7_2_3_Model.CreateGeometryModel(model, numElements, outputPlotDirectory);
				FriesExample_7_2_3_Model.SetupEnrichmentOutput(model, outputPlotDirectory);
			}
			else
			{
				FriesExample_7_2_3_Model.CreateGeometryModel(model, numElements);
			}

			//SolverChoice solverChoice = SolverChoice.FetiDPManaged;
			SolverChoice solverChoice = SolverChoice.FetiDPNative;
			(ISolver solver, IAlgebraicModel algebraicModel, DdmLogger logger)
				= SetupFetiDPSolver(model, nodeTopology, solverChoice, numSubdomains);
			RunAnalysis(model, algebraicModel, solver, solverChoice);


			string path = Path.Combine(outputDirectory, "fetidp_convergence.txt");
			logger.WriteToFile(path, true);
		}

		[Fact]
		public static void RunExampleWithPFetiDPSolver()
		{
			int[] numClusters = { 1, 1, 1 };
			(XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
					= FriesExample_7_2_3_Model.DescribePhysicalModel(numElements, numSubdomains, numClusters)
					.BuildMultiSubdomainModel();
			if (enablePlotting)
			{
				FriesExample_7_2_3_Model.CreateGeometryModel(model, numElements, outputPlotDirectory);
				FriesExample_7_2_3_Model.SetupEnrichmentOutput(model, outputPlotDirectory);
			}
			else
			{
				FriesExample_7_2_3_Model.CreateGeometryModel(model, numElements);
			}

			//SolverChoice solverChoice = SolverChoice.PfetiDPManaged;
			SolverChoice solverChoice = SolverChoice.PfetiDPNative;
			(ISolver solver, IAlgebraicModel algebraicModel, DdmLogger logger) 
				= SetupPFetiDPSolver(model, nodeTopology, solverChoice, numSubdomains);
			RunAnalysis(model, algebraicModel, solver, solverChoice);

			string path = Path.Combine(outputDirectory, "pfetidp_convergence.txt");
			logger.WriteToFile(path, true);
		}

		[Fact]
		public static void RunExampleWithReanalysisSolver()
		{
			XModel<IXCrackElement> model = FriesExample_7_2_3_Model.DescribePhysicalModel(numElements).BuildSingleSubdomainModel();
			if (enablePlotting)
			{
				FriesExample_7_2_3_Model.CreateGeometryModel(model, numElements, outputPlotDirectory);
				FriesExample_7_2_3_Model.SetupEnrichmentOutput(model, outputPlotDirectory);
			}
			else
			{
				FriesExample_7_2_3_Model.CreateGeometryModel(model, numElements);
			}
			SolverChoice solverChoice = SolverChoice.DirectReanalysis;
			
			// Possibly enriched nodes
			var crackGeometries = new List<IImplicitCrackGeometry>();
			ICrack fullCrack = FriesExample_7_2_3_Model.CreateFullCrack(model, maxIterations);
			crackGeometries.Add((IImplicitCrackGeometry)(fullCrack.CrackGeometry));
			double dx = (FriesExample_7_2_3_Model.maxCoords[0] - FriesExample_7_2_3_Model.minCoords[0]) / numElements[0];
			double dy = (FriesExample_7_2_3_Model.maxCoords[1] - FriesExample_7_2_3_Model.minCoords[1]) / numElements[1];
			double dz = (FriesExample_7_2_3_Model.maxCoords[2] - FriesExample_7_2_3_Model.minCoords[2]) / numElements[2];
			double maxDistance = 2 * Math.Sqrt(dx * dx + dy * dy + dz * dz); // Enriched nodes are at most 2 elements away from the crack.
			var enrichedNodeSelector = new CrackVicinityNodeSelector(crackGeometries, maxDistance);
			//var enrichedNodeSelector = new BoundingBoxNodeSelector(new double[] { 300, 0, 0 }, new double[] { 431.25, 150, 75 });

			var dofOrderer = new ReanalysisDofOrderer(enrichedNodeSelector.CanNodeBeEnriched);
			var factory = new ReanalysisRebuildingSolver.Factory(dofOrderer);
			if (reanalysisExtraDofs == ReanalysisExtraDofs.None)
			{
				factory.ExtraDofsStrategy = new NoExtraModifiedDofsStrategy();
			}
			else if (reanalysisExtraDofs == ReanalysisExtraDofs.CrackStepNearModified)
			{
				factory.ExtraDofsStrategy = new StepDofsNearModifiedNodesStrategy();
			}
			else if (reanalysisExtraDofs == ReanalysisExtraDofs.LimitedNearModified)
			{
				factory.ExtraDofsStrategy = new LimitedDofsNearModifiedDofsStategy();
			}
			else
			{
				factory.ExtraDofsStrategy = new AllDofsNearModifiedDofsStrategy();
			}
			ReanalysisAlgebraicModel<DokMatrixAdapter> algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);

			RunAnalysis(model, algebraicModel, solver, solverChoice);
		}

		private static void RunAnalysis(XModel<IXCrackElement> model, IAlgebraicModel algebraicModel, ISolver solver, 
			SolverChoice solverChoice)
		{
			var domainBoundary = new RectangularDomainBoundary(
				FriesExample_7_2_3_Model.minCoords, FriesExample_7_2_3_Model.maxCoords);
			var termination = new TerminationLogic.Or(
				new FractureToughnessTermination(fractureToughness),
				new CrackExitsDomainTermination(domainBoundary));
			var analyzer = new QuasiStaticLefmAnalyzer(model, algebraicModel, solver, maxIterations, termination);

			var msg = new StringBuilder();
			msg.Append($"{DateTime.Now}, solver={solverChoice.ToString()}");
			msg.AppendLine($", numElements={numElements[0]}x{numElements[1]}x{numElements[2]}," +
				$" heaviside tol={FriesExample_7_2_3_Model.heavisideTol}, tip enrichment radius={FriesExample_7_2_3_Model.tipEnrichmentArea}");
			if (solverChoice == SolverChoice.PfetiDPManaged || solverChoice == SolverChoice.PfetiDPNative
				|| solverChoice == SolverChoice.FetiDPManaged || solverChoice == SolverChoice.FetiDPNative)
			{
				msg.Append($"numSubdomains={numSubdomains[0]}x{numSubdomains[1]}x{numSubdomains[2]}");
				msg.AppendLine($", reanalysis={ddmReanalysis}, multithreaded environment={multiThreaded}, PSM tolerance={iterTol}");
			}
			else if (solverChoice == SolverChoice.DirectReanalysis)
			{
				msg.AppendLine($"reanalysis extra modified dofs={reanalysisExtraDofs}");
			}
			if (solverChoice == SolverChoice.FetiDPManaged || solverChoice == SolverChoice.FetiDPNative)
			{
				msg.AppendLine($"FETI-DP preconditioner = {preconditionerFetiDP}");
			}

			var normLogger = new SolutionNormLogger(Path.Combine(outputDirectory, "solution_norm.txt"));
			normLogger.ExtraInfo = msg.ToString();
			analyzer.Results.Add(normLogger);
			analyzer.Logger.ExtraInfo = msg.ToString();
			solver.Logger.ExtraInfo = msg.ToString();
			//analyzer.Results.Add(new StructuralFieldWriter(model, outputDirectory));

			Console.WriteLine("Starting analysis");
			analyzer.Analyze();

			string performanceOutputFile = Path.Combine(outputDirectory, "performance.txt");
			//analyzer.Logger.WriteToFile(performanceOutputFile, true);
			solver.Logger.WriteAggregatesToFile(performanceOutputFile, true);
			solver.Logger.WriteToFile(performanceOutputFile, true);
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

		private static (ISolver, IAlgebraicModel, DdmLogger) SetupFetiDPSolver(XModel<IXCrackElement> model,
			ComputeNodeTopology nodeTopology, SolverChoice solverChoice, int[] numSubdomains)
		{
			// Environment
			IComputeEnvironment environment;
			if (multiThreaded)
			{
				environment = new TplSharedEnvironment(true);
			}
			else
			{
				environment = new SequentialSharedEnvironment(true);
			}
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
				//double[] minCoords = FriesExample_7_2_3_Model.minCoords;
				//double[] maxCoords = FriesExample_7_2_3_Model.maxCoords;
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
			IFetiDPSubdomainMatrixManagerFactory<SymmetricCscMatrix> fetiDPMatrices;
			IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix;
			if (solverChoice == SolverChoice.FetiDPManaged)
			{
				fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricCSparse.Factory(false);
				coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCSparse();
			}
			else if (solverChoice == SolverChoice.FetiDPNative)
			{
				if (unsafeOptimizations)
				{
					//fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricSuiteSparseUnsafe.Factory(false);
					fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricSuiteSparse.Factory(false);

				}
				else
				{
					fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricSuiteSparse.Factory(false);
				}
				coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricSuiteSparse();
			}
			else
			{
				throw new NotImplementedException();
			}

			var solverFactory = new FetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, cornerDofs, fetiDPMatrices);
			solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
			solverFactory.EnableLogging = true;
			solverFactory.ExplicitSubdomainMatrices = false;
			solverFactory.InterfaceProblemSolverFactory = new FetiDPInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = iterTol,
				UseObjectiveConvergenceCriterion = objectivePcgCriterion
			};
			if (preconditionerFetiDP == PreconditionerFetiDP.Dirichlet)
			{
				solverFactory.Preconditioner = new FetiDPDirichletPreconditioner();
			}
			else if (preconditionerFetiDP == PreconditionerFetiDP.DiagonalDirichlet)
			{
				solverFactory.Preconditioner = new FetiDPDiagonalDirichletPreconditioner();
			}
			else
			{
				solverFactory.Preconditioner = new FetiDPLumpedPreconditioner();
			}


			if (ddmReanalysis)
			{
				var observer = new SubdomainEnrichmentsModifiedObserver();
				model.GeometryModel.Enricher.Observers.Add(observer);
				var reanalysisOptions = FetiDPReanalysisOptions.CreateWithAllEnabled(observer);
				//var reanalysisOptions = PFetiDPReanalysisOptions.CreateWithAllDisabled();
				reanalysisOptions.PreviousSolution = false; // This causes errors if enabled

				solverFactory.ReanalysisOptions = reanalysisOptions;
				solverFactory.SubdomainTopology = new SubdomainTopologyOptimized();
				solverFactory.ExplicitSubdomainMatrices = false;
			}

			// Create solver
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(model, algebraicModel);
			return (solver, algebraicModel, solver.LoggerDdm);
		}

		private static (ISolver, IAlgebraicModel, DdmLogger) SetupPFetiDPSolver(XModel<IXCrackElement> model, 
			ComputeNodeTopology nodeTopology, SolverChoice solverChoice, int[] numSubdomains)
		{
			// Environment
			IComputeEnvironment environment;
			if (multiThreaded)
			{
				environment = new TplSharedEnvironment(true);
			}
			else
			{
				environment = new SequentialSharedEnvironment(true);
			}
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
				//double[] minCoords = FriesExample_7_2_3_Model.minCoords;
				//double[] maxCoords = FriesExample_7_2_3_Model.maxCoords;
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
				fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricCSparse.Factory(true);
				coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCSparse();
			}
			else if (solverChoice == SolverChoice.PfetiDPNative)
			{
				if (unsafeOptimizations)
				{
					psmMatrices = new PsmSubdomainMatrixManagerSymmetricSuiteSparseUnsafe.Factory();
					fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricSuiteSparse.Factory(true);
				}
				else
				{
					psmMatrices = new PsmSubdomainMatrixManagerSymmetricSuiteSparse.Factory();
					fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricSuiteSparse.Factory(true);
				}
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
			solverFactory.ExplicitSubdomainMatrices = explicitPsmMatrices;
			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = iterTol,
				UseObjectiveConvergenceCriterion = objectivePcgCriterion
			};

			if (ddmReanalysis)
			{
				var observer = new SubdomainEnrichmentsModifiedObserver();
				model.GeometryModel.Enricher.Observers.Add(observer);
				var reanalysisOptions = PFetiDPReanalysisOptions.CreateWithAllEnabled(observer);
				//var reanalysisOptions = PFetiDPReanalysisOptions.CreateWithAllDisabled();
				reanalysisOptions.PreviousSolution = false; // This causes errors if enabled

				solverFactory.ReanalysisOptions = reanalysisOptions;
				solverFactory.SubdomainTopology = new SubdomainTopologyOptimized();
				solverFactory.ExplicitSubdomainMatrices = explicitPsmMatrices;
			}

			// Create solver
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(model, algebraicModel);
			return (solver, algebraicModel, solver.LoggerDdm);
		}
	}
}
