using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.DDM;
using MGroup.Solvers.DDM.FetiDP;
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.InterfaceProblem;
using MGroup.Solvers.DDM.FetiDP.Preconditioning;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.Output;
using MGroup.Solvers.DDM.PFetiDP;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.XFEM.Analysis;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Boundaries;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Solvers.PFetiDP;
using MGroup.XFEM.Tests.SpecialSolvers.HybridFries;

namespace MGroup.XFEM.Tests.SpecialSolvers
{
	public static class ExamplesMpi3D
	{
		public enum SolverType
		{
			PCG, FETI_DP, PFETI_DP
		}

		public enum SolverChoice 
		{ 
			PCG_D, FETI_DP_D, FETI_DP_D_I, FETI_DP_L, FETI_DP_L_I, PFETI_DP, PFETI_DP_I
		}

		public class ExampleOptions
		{
			public double heavisideTol = 1E-4;
			public int maxSteps = -1;
		}

		public class ExampleBB4POptions : ExampleOptions
		{
			public double crackFrontX = 337.5;
			public double crackFrontY = 74;

			public ExampleBB4POptions(double crackMouthX)
			{
				this.crackFrontX = crackMouthX;
				maxSteps = 13;
			}
		}

		public class ExampleImpactOptions : ExampleOptions
		{
			public ExampleImpactOptions()
			{
				maxSteps = 16;
			}
		}

		public class MeshOptions
		{
			public readonly int minElements;
			public readonly int minSubdomains;

			public int[] numClusters = { 1, 1, 1};
			public int[] numElements;
			public int[] numSubdomains;

			public MeshOptions(int minElements, int minSubdomains)
			{
				this.minElements = minElements;
				this.minSubdomains = minSubdomains;
			}
		}

		public class OutputOptions
		{
			public readonly string outputDirectoryForced;
			private readonly bool mgroupCluster;
			private readonly string investigationName;

			public OutputOptions(string outputDirectory)
			{
				this.outputDirectoryForced = outputDirectory;
			}

			public OutputOptions(bool mgroupCluster, string investigationName)
			{
				this.mgroupCluster = mgroupCluster;
				this.investigationName = investigationName;
			}

			public string GetOutputDirectory(ExampleOptions exampleOptions)
			{
				if (outputDirectoryForced != null)
				{
					return outputDirectoryForced;
				}

				string directory;
				if (mgroupCluster)
				{
					directory = @"C:\Users\cluster\Desktop\Serafeim\results\MPI\";
				}
				else
				{
					directory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\MPI\";
				}

				if (exampleOptions is ExampleBB4POptions)
				{
					directory += @"Example1\";
				}
				else
				{
					directory += @"Example2\";
				}

				directory += $"{investigationName}\\";

				return directory;
			}
		}

		public class SolverOptions
		{
			public readonly SolverChoice solverChoice;
			public readonly SolverType solverType;

			public bool objectiveConvergenceCriterion = true;
			public double pcgTolerance = 1E-6;
			public int maxPcgIterations = 600;

			public bool managedDirectSolvers = false;
			public bool explicitSchurComplements = false;
			public bool unsafeOptimizations = true;

			public bool multiThreaded = true;

			public SolverOptions(SolverChoice solverChoice)
			{
				this.solverChoice = solverChoice;
				if (solverChoice == SolverChoice.PCG_D)
				{
					this.solverType = SolverType.PCG;
				}
				else if (solverChoice == SolverChoice.PFETI_DP || solverChoice == SolverChoice.PFETI_DP)
				{
					this.solverType = SolverType.PFETI_DP;
				}
				else
				{
					this.solverType = SolverType.FETI_DP;
				}
			}
		}

		public static void RunSingleAnalysis(ExampleOptions exampleOptions, MeshOptions meshOptions, SolverOptions solverOptions)
		{
			XModel<IXCrackElement> model;
			RectangularDomainBoundary boundary;
			ComputeNodeTopology nodeTopology;
			if (exampleOptions is ExampleBB4POptions)
			{
				(model, boundary, nodeTopology) = 
					SetupExampleBB4P((ExampleBB4POptions)exampleOptions, meshOptions, solverOptions);
			}
			else
			{
				(model, boundary, nodeTopology) = 
					SetupExampleImpact((ExampleImpactOptions)exampleOptions, meshOptions, solverOptions);
			}

			(ISolver solver, IAlgebraicModel algebraicModel, DdmLogger loggerDdm) 
				= SetupSolver(meshOptions, solverOptions, model, nodeTopology);

			HERΕ
		}

		private static string CreateHeader(ExampleOptions exampleOptions, MeshOptions meshOptions, SolverOptions solverOptions)
		{
			var msg = new StringBuilder();
			msg.Append($"New analysis, {DateTime.Now}");

			msg.Append($"{DateTime.Now}, solver={solverChoice.ToString()}");
			msg.AppendLine($", numElements={numElements[0]}x{numElements[1]}x{numElements[2]}, poisson ratio={FriesExample_7_2_3_Model.v}" +
				$" heaviside tol={FriesExample_7_2_3_Model.heavisideTol}, tip enrichment radius={FriesExample_7_2_3_Model.tipEnrichmentArea}");
			if (solverChoice == SolverChoice.PfetiDPManaged || solverChoice == SolverChoice.PfetiDPNative
				|| solverChoice == SolverChoice.FetiDPManaged || solverChoice == SolverChoice.FetiDPNative)
			{
				msg.Append($"numSubdomains={numSubdomains[0]}x{numSubdomains[1]}x{numSubdomains[2]}");
				msg.AppendLine($", reanalysis={ddmReanalysis}, multithreaded environment={multiThreaded}, PSM tolerance={iterTol}");
				msg.AppendLine($", corner dofs strategy = {CrackFetiDPCornerDofs.strategy}");
			}
			else if (solverChoice == SolverChoice.DirectReanalysis)
			{
				msg.AppendLine($"reanalysis extra modified dofs={reanalysisExtraDofs}");
			}
			else if (solverChoice == SolverChoice.PcgJacobi)
			{
				msg.AppendLine($"iterative tolerance={iterTol}");
			}

			if (solverChoice == SolverChoice.FetiDPManaged || solverChoice == SolverChoice.FetiDPNative)
			{
				msg.AppendLine($"FETI-DP preconditioner = {preconditionerFetiDP}");
			}
		}

		private static void RunAnalysis(
			ExampleOptions exampleOptions, MeshOptions meshOptions, SolverOptions solverOptions, OutputOptions outputOptions,
			XModel<IXCrackElement> model, RectangularDomainBoundary domainBoundary, 
			IAlgebraicModel algebraicModel, ISolver solver)
		{
			var termination = new TerminationLogic.Or(
				new FractureToughnessTermination(double.MaxValue),
				new CrackExitsDomainTermination(domainBoundary));
			var analyzer = new QuasiStaticLefmAnalyzer(model, algebraicModel, solver, exampleOptions.maxSteps, termination);

			string outputDirectory = outputOptions.GetOutputDirectory(exampleOptions);
			var normLogger = new SolutionNormLogger(Path.Combine(outputDirectory, "solution_norm.txt"));
			string header = CreateHeader();
			normLogger.ExtraInfo = header;
			analyzer.Results.Add(normLogger);
			analyzer.Logger.ExtraInfo = header;
			solver.Logger.ExtraInfo = header;
			//analyzer.Results.Add(new StructuralFieldWriter(model, outputDirectory));

			Console.WriteLine("Starting analysis");
			analyzer.Analyze();

			string performanceOutputFile = Path.Combine(outputDirectory, "performance.txt");
			//analyzer.Logger.WriteToFile(performanceOutputFile, true);
			solver.Logger.WriteAggregatesToFile(performanceOutputFile, true);
			solver.Logger.WriteToFile(performanceOutputFile, true);
		}

		public static (XModel<IXCrackElement>, RectangularDomainBoundary, ComputeNodeTopology) SetupExampleBB4P(
			ExampleBB4POptions exampleOptions, MeshOptions meshOptions, SolverOptions solverOptions)
		{
			meshOptions.numElements = new int[] { 9 * meshOptions.minElements, 2 * meshOptions.minElements, meshOptions.minElements };
			meshOptions.numSubdomains = new int[] { 9 * meshOptions.minSubdomains, 2 * meshOptions.minSubdomains, meshOptions.minSubdomains };

			XModel<IXCrackElement> model;
			ComputeNodeTopology nodeTopology;
			if (solverOptions.solverType == SolverType.PCG)
			{
				model = FriesExample_7_2_1_Model.DescribePhysicalModel(meshOptions.numElements).BuildSingleSubdomainModel();
				nodeTopology = null;
			}
			else
			{
				(model, nodeTopology) = FriesExample_7_2_1_Model.DescribePhysicalModel(
					meshOptions.numElements, meshOptions.numSubdomains, meshOptions.numClusters).BuildMultiSubdomainModel();
			}

			double[] crackMouthCoords = { exampleOptions.crackFrontX, 0 };
			double[] crackFrontCoords = { exampleOptions.crackFrontX, exampleOptions.crackFrontY };
			FriesExample_7_2_1_Model.CreateGeometryModel(model, meshOptions.numElements, crackMouthCoords, crackFrontCoords);
			FriesExample_7_2_1_Model.heavisideTol = exampleOptions.heavisideTol;
			
			var domainBoundary = new RectangularDomainBoundary(
				FriesExample_7_2_1_Model.minCoords, FriesExample_7_2_1_Model.maxCoords);

			return (model, domainBoundary, nodeTopology);
		}

		public static (XModel<IXCrackElement>, RectangularDomainBoundary, ComputeNodeTopology) SetupExampleImpact(
			ExampleImpactOptions exampleOptions, MeshOptions meshOptions, SolverOptions solverOptions)
		{
			meshOptions.numElements = new int[] { 2 * meshOptions.minElements, meshOptions.minElements, 2 * meshOptions.minElements };
			meshOptions.numSubdomains = new int[] { 2 * meshOptions.minSubdomains, meshOptions.minSubdomains, 2 * meshOptions.minSubdomains };


			XModel<IXCrackElement> model;
			ComputeNodeTopology nodeTopology;
			if (solverOptions.solverType == SolverType.PCG)
			{
				model = FriesExample_7_2_3_Model.DescribePhysicalModel(meshOptions.numElements).BuildSingleSubdomainModel();
				nodeTopology = null;
			}
			else
			{
				(model, nodeTopology) = FriesExample_7_2_3_Model.DescribePhysicalModel(
					meshOptions.numElements, meshOptions.numSubdomains, meshOptions.numClusters).BuildMultiSubdomainModel();
			}

			FriesExample_7_2_3_Model.CreateGeometryModel(model, meshOptions.numElements);
			FriesExample_7_2_3_Model.heavisideTol = exampleOptions.heavisideTol;

			var domainBoundary = new RectangularDomainBoundary(
				FriesExample_7_2_3_Model.minCoords, FriesExample_7_2_3_Model.maxCoords);

			return (model, domainBoundary, nodeTopology);
		}

		public static (ISolver, IAlgebraicModel, DdmLogger) SetupSolver(
			MeshOptions meshOptions, SolverOptions solverOptions, XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
		{
			if (solverOptions.solverType == SolverType.PCG)
			{
				return SetupPcgSolver();
			}
			else if (solverOptions.solverType == SolverType.FETI_DP)
			{
				return SetupFetiDPSolver(meshOptions, solverOptions, model, nodeTopology);
			}
			else if (solverOptions.solverType == SolverType.PFETI_DP)
			{
				return SetupPFetiDPSolver(meshOptions, solverOptions, model, nodeTopology);
			}
			else
			{
				throw new NotImplementedException();
			}
		}

		public static (ISolver, IAlgebraicModel, DdmLogger) SetupPcgSolver()
		{
			throw new NotImplementedException();
		}


		public static (ISolver, IAlgebraicModel, DdmLogger) SetupFetiDPSolver(
			MeshOptions meshOptions, SolverOptions solverOptions, XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
		{
			IComputeEnvironment environment = CreateEnvironment(solverOptions, nodeTopology);
			ICornerDofSelection cornerDofs = CreateCornerDofs(meshOptions, environment, model);

			// Solver settings
			IFetiDPSubdomainMatrixManagerFactory<SymmetricCscMatrix> fetiDPMatrices;
			IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix;
			if (solverOptions.managedDirectSolvers)
			{
				fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricCSparse.Factory(false);
				coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCSparse();
			}
			else
			{
				if (solverOptions.unsafeOptimizations)
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

			var solverFactory = new FetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, cornerDofs, fetiDPMatrices);
			solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
			solverFactory.EnableLogging = true;
			solverFactory.ExplicitSubdomainMatrices = false;
			solverFactory.InterfaceProblemSolverFactory = new FetiDPInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = solverOptions.maxPcgIterations,
				ResidualTolerance = solverOptions.pcgTolerance,
				UseObjectiveConvergenceCriterion = solverOptions.objectiveConvergenceCriterion
			};
			MGroup.Solvers.DDM.FetiDP.InterfaceProblem.ObjectiveConvergenceCriterion<SymmetricCscMatrix>.optimizationsForSymmetricCscMatrix = true;

			if (solverOptions.solverChoice == SolverChoice.FETI_DP_D || solverOptions.solverChoice == SolverChoice.FETI_DP_D_I)
			{
				solverFactory.Preconditioner = new FetiDPDirichletPreconditioner();
			}
			else if (solverOptions.solverChoice == SolverChoice.FETI_DP_L || solverOptions.solverChoice == SolverChoice.FETI_DP_L_I)
			{
				solverFactory.Preconditioner = new FetiDPLumpedPreconditioner();
			}
			else
			{
				throw new NotImplementedException();
				//solverFactory.Preconditioner = new FetiDPDiagonalDirichletPreconditioner();
			}

			if (solverOptions.solverChoice == SolverChoice.FETI_DP_D_I || solverOptions.solverChoice == SolverChoice.FETI_DP_L_I)
			{
				var observer = new SubdomainEnrichmentsModifiedObserver();
				model.GeometryModel.Enricher.Observers.Add(observer);
				var reanalysisOptions = FetiDPReanalysisOptions.CreateWithAllEnabled(observer);
				//var reanalysisOptions = FetiDPReanalysisOptions.CreateWithAllDisabled();
				reanalysisOptions.PreviousSolution = true;

				solverFactory.ReanalysisOptions = reanalysisOptions;
				solverFactory.SubdomainTopology = new SubdomainTopologyOptimized();
				solverFactory.ExplicitSubdomainMatrices = false;
			}

			// Create solver
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(model, algebraicModel);
			return (solver, algebraicModel, solver.LoggerDdm);
		}

		public static (ISolver, IAlgebraicModel, DdmLogger) SetupPFetiDPSolver(
			MeshOptions meshOptions, SolverOptions solverOptions, XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
		{
			IComputeEnvironment environment = CreateEnvironment(solverOptions, nodeTopology);
			ICornerDofSelection cornerDofs = CreateCornerDofs(meshOptions, environment, model);

			// Solver settings
			IPsmSubdomainMatrixManagerFactory<SymmetricCscMatrix> psmMatrices;
			IFetiDPSubdomainMatrixManagerFactory<SymmetricCscMatrix> fetiDPMatrices;
			IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix;
			if (solverOptions.managedDirectSolvers)
			{
				psmMatrices = new PsmSubdomainMatrixManagerSymmetricCSparse.Factory();
				fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricCSparse.Factory(true);
				coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCSparse();
			}
			else
			{
				if (solverOptions.unsafeOptimizations)
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

			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, psmMatrices, cornerDofs, fetiDPMatrices);
			solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
			solverFactory.EnableLogging = true;
			solverFactory.ExplicitSubdomainMatrices = solverOptions.explicitSchurComplements;
			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = solverOptions.maxPcgIterations,
				ResidualTolerance = solverOptions.pcgTolerance,
				UseObjectiveConvergenceCriterion = solverOptions.objectiveConvergenceCriterion
			};
			MGroup.Solvers.DDM.PSM.InterfaceProblem.ObjectiveConvergenceCriterion<SymmetricCscMatrix>.optimizationsForSymmetricCscMatrix = true;


			if (solverOptions.solverChoice == SolverChoice.PFETI_DP_I)
			{
				var observer = new SubdomainEnrichmentsModifiedObserver();
				model.GeometryModel.Enricher.Observers.Add(observer);
				var reanalysisOptions = PFetiDPReanalysisOptions.CreateWithAllEnabled(observer);
				//var reanalysisOptions = PFetiDPReanalysisOptions.CreateWithAllDisabled();
				reanalysisOptions.PreviousSolution = true;

				solverFactory.ReanalysisOptions = reanalysisOptions;
				solverFactory.SubdomainTopology = new SubdomainTopologyOptimized();
				solverFactory.ExplicitSubdomainMatrices = solverOptions.explicitSchurComplements;
			}

			// Create solver
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(model, algebraicModel);
			return (solver, algebraicModel, solver.LoggerDdm);
		}

		private static ICornerDofSelection CreateCornerDofs(
			MeshOptions meshOptions, IComputeEnvironment environment, XModel<IXCrackElement> model)
		{
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			IDofType[] stdDofs = { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ };

			int minMultiplicity = 3;
			if (meshOptions.numSubdomains[2] == 1)
			{
				minMultiplicity = 2;
			}
			var cornerDofs = new CrackFetiDPCornerDofsPlusLogging(environment, model, stdDofs,
				sub => UniformDdmCrackModelBuilder3D.FindCornerNodes(sub, minMultiplicity));

			CrackFetiDPCornerDofs.strategy = CrackFetiDPCornerDofs.Strategy.HeavisideAndAllTipDofs;

			return cornerDofs;
		}

		private static IComputeEnvironment CreateEnvironment(SolverOptions solverOptions, ComputeNodeTopology nodeTopology)
		{
			IComputeEnvironment environment;
			if (solverOptions.multiThreaded)
			{
				environment = new TplSharedEnvironment(true);
			}
			else
			{
				environment = new SequentialSharedEnvironment(true);
			}
			environment.Initialize(nodeTopology);
			return environment;
		}
	}
}
