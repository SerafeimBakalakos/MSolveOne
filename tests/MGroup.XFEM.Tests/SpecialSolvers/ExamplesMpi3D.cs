using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.Environments;
using MGroup.Environments.Mpi;
using MGroup.LinearAlgebra.Distributed.IterativeMethods.PCG;
using MGroup.LinearAlgebra.Distributed.IterativeMethods.PCG.Reorthogonalization;
using MGroup.LinearAlgebra.Iterative.Termination;
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
		public enum EnvironmentChoice
		{
			Serial, TPL, MPI
		}

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
			public double poissonRatio = 0.3;
			public double tipEnrichmentRadius = 0.0;

			public virtual string[] GetOtherOptions() => new string[0];
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

			public override string[] GetOtherOptions()
			{
				return new string[] { $"crack front ({crackFrontX}, {crackFrontY})" };
			}

			public override string ToString() => "4PBB";
		}

		public class ExampleImpactOptions : ExampleOptions
		{
			public ExampleImpactOptions()
			{
				maxSteps = 16;
			}

			public override string ToString() => "Impact";
		}

		public class MeshOptions
		{
			public readonly int minElements;
			public readonly int minSubdomains;

			public int[] numClusters = { 1, 1, 1 };
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

			public string GetParentOutputDirectory()
			{
				if (mgroupCluster)
				{
					return @"C:\Users\cluster\Desktop\Serafeim\";
				}
				else
				{
					return @"C:\Users\Serafeim\Desktop\xfem 3d\paper\";
				}
			}

			public string GetOutputDirectory(ExampleOptions exampleOptions, SolverOptions solverOptions)
			{
				if (outputDirectoryForced != null)
				{
					return outputDirectoryForced;
				}

				string directory = GetParentOutputDirectory();
				directory += @"results\MPI\";

				if (exampleOptions is ExampleBB4POptions)
				{
					directory += @"Example1\";
				}
				else
				{
					directory += @"Example2\";
				}

				directory += $"{investigationName}\\";
				if (solverOptions.environmentChoice == EnvironmentChoice.MPI)
				{
					directory += $"process{MPI.Communicator.world.Rank}\\";
				}

				return directory;
			}

			public void CreateDirectory(ExampleOptions exampleOptions, SolverOptions solverOptions)
			{
				if (outputDirectoryForced != null)
				{
					return;
				}

				// This must exist
				string parentDirectory = GetParentOutputDirectory();
				if (!Directory.Exists(parentDirectory))
				{
					throw new IOException($"Cannot proceed if the parent directory \"{parentDirectory}\" does not exist.");
				}

				string directory = GetOutputDirectory(exampleOptions, solverOptions);
				Directory.CreateDirectory(directory);
			}
		}

		public class SolverOptions
		{
			public readonly SolverChoice solverChoice;
			public readonly SolverType solverType;

			public bool objectiveConvergenceCriterion = true;
			public double pcgTolerance = 1E-7;
			public int maxPcgIterations = 600;

			public bool managedDirectSolvers = false;
			public bool explicitSchurComplements = false;
			public bool unsafeOptimizations = true;

			public EnvironmentChoice environmentChoice = EnvironmentChoice.TPL;
			public MpiEnvironment environment;

			public CrackFetiDPCornerDofs.Strategy cornerDofStrategy = CrackFetiDPCornerDofs.Strategy.HeavisideAndAllTipDofs;

			public SolverOptions(SolverChoice solverChoice)
			{
				this.solverChoice = solverChoice;
				if (solverChoice == SolverChoice.PCG_D)
				{
					this.solverType = SolverType.PCG;
				}
				else if (solverChoice == SolverChoice.PFETI_DP || solverChoice == SolverChoice.PFETI_DP_I)
				{
					this.solverType = SolverType.PFETI_DP;
				}
				else
				{
					this.solverType = SolverType.FETI_DP;
				}
			}
		}

		public class CoarseProblemOptions
		{
			public readonly int numClusters;
			public readonly int numProcesses;

			public CoarseProblemOptions(int numClusters, int numProcesses)
			{
				this.numClusters = numClusters;
				this.numProcesses = numProcesses;
			}
		}

		public class CoarseProblemOptionsGlobal : CoarseProblemOptions
		{
			public readonly List<int> masterProcesses = new List<int>();
			public bool Democratic { get; private set; }

			public int[] MasterProcesses => masterProcesses.ToArray();

			public CoarseProblemOptionsGlobal(int numClusters, int numProcesses) : base(numClusters, numProcesses)
			{
				// Set the last one as master
				masterProcesses.Add(numProcesses - 1);
				Democratic = false;
			}

			public void SetMasterProcess(int proc)
			{
				if (proc > numClusters)
				{
					throw new ArgumentException("The master must be one of the active processes or the one after them");
				}
				masterProcesses.Clear();
				masterProcesses.Add(proc);
			}

			public void ForceDemocracy()
			{
				Democratic = true;
				masterProcesses.Clear();
				for (int c = 0; c< numClusters; ++c)
				{
					masterProcesses.Add(c);
				}
			}

			public override string ToString()
			{
				var msg = new StringBuilder();
				if (numProcesses == 1)
				{
					msg.Append("Global single machine, ");
				}
				else if (Democratic)
				{
					msg.Append("Global democratic, ");
				}
				else if (numProcesses == numClusters)
				{
					msg.Append("Global master-slaves, ");
				}
				else
				{
					msg.Append("Global slaves-extra master, ");
				}
				msg.Append($"num clusters = {numClusters}, num processes = {numProcesses}, master processes=[ ");
				foreach (int p in MasterProcesses)
				{
					msg.Append($"{p} ");
				}
				msg.Append("]");

				return msg.ToString();
			}
		}

		public class CoarseProblemOptionsDistributed : CoarseProblemOptions
		{
			public bool jacobiPreconditioner = true;
			public double pcgTol = 1E-7;
			public int pcgMaxIter = 200;
			public bool reorthoPCG = false;
			public int reorthoCacheSize = 30;

			public CoarseProblemOptionsDistributed(int numClusters, int numProcesses) : base(numClusters, numProcesses)
			{
			}

			public override string ToString()
			{
				string precond = jacobiPreconditioner ? "Jacobi" : "none";
				var msg = new StringBuilder();
				if (reorthoPCG)
				{
					msg.Append("Distributed reortho-PCG, ");
					msg.Append($"pcg tol={pcgTol}, pcg max iterations={pcgMaxIter}, preconditioner={precond}, ");
					msg.Append($"reortho cache size={reorthoCacheSize}");
				}
				else
				{
					msg.Append("Distributed PCG, ");
					msg.Append($"pcg tol={pcgTol}, pcg max iterations={pcgMaxIter}, preconditioner={precond}");
				}
				return msg.ToString();
			}
		}

		public static void RunSingleAnalysis(ExampleOptions exampleOptions, MeshOptions meshOptions, SolverOptions solverOptions, 
			OutputOptions outputOptions, CoarseProblemOptions coarseProblemOptions)
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
				= SetupSolver(meshOptions, solverOptions, coarseProblemOptions, model, nodeTopology);

			RunAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions, coarseProblemOptions, 
				model, boundary, algebraicModel, solver, loggerDdm);
		}

		public static string CreateHeader(ExampleOptions exampleOptions, MeshOptions meshOptions, SolverOptions solverOptions,
			CoarseProblemOptions coarseProblemOptions)
		{
			var msg = new StringBuilder();
			msg.AppendLine($"New analysis, {DateTime.Now}");
			msg.Append($"Example={exampleOptions}, num steps={exampleOptions.maxSteps}, poisson={exampleOptions.poissonRatio}, " +
				$"heaviside tol={exampleOptions.heavisideTol}, tip radius={exampleOptions.tipEnrichmentRadius}");
			if (exampleOptions.GetOtherOptions().Length >= 1)
			{
				msg.Append(", " + exampleOptions.GetOtherOptions());
			}
			msg.AppendLine();

			msg.AppendLine($"Num elements={meshOptions.numElements[0]}x{meshOptions.numElements[1]}x{meshOptions.numElements[2]}, " +
				$"num subdomains={meshOptions.numSubdomains[0]}x{meshOptions.numSubdomains[1]}x{meshOptions.numSubdomains[2]}, " +
				$"num clusters={meshOptions.numClusters[0]}x{meshOptions.numClusters[1]}x{meshOptions.numClusters[2]}, " +
				$"H/h = {meshOptions.minElements / (double)(meshOptions.minSubdomains)}");

			msg.AppendLine($"Solver={solverOptions.solverChoice}, corner dof strategy={solverOptions.cornerDofStrategy}, " +
				$"pcg tol={solverOptions.pcgTolerance}, objective criterion={solverOptions.objectiveConvergenceCriterion}, " +
				$"managed direct solvers={solverOptions.managedDirectSolvers}, unsafe optimizations={solverOptions.unsafeOptimizations}, " +
				$"explicit Schur complements={solverOptions.explicitSchurComplements}");

			msg.AppendLine($"Environment={solverOptions.environmentChoice}, num processes={coarseProblemOptions.numProcesses}");
			msg.AppendLine("Coarse problem strategy: " +coarseProblemOptions.ToString());

			return msg.ToString();
		}

		private static void RunAnalysis(ExampleOptions exampleOptions, MeshOptions meshOptions, SolverOptions solverOptions, 
			OutputOptions outputOptions, CoarseProblemOptions coarseProblemOptions,
			XModel<IXCrackElement> model, RectangularDomainBoundary domainBoundary, 
			IAlgebraicModel algebraicModel, ISolver solver, DdmLogger ddmLogger)
		{
			var termination = new TerminationLogic.Or(
				new FractureToughnessTermination(double.MaxValue),
				new CrackExitsDomainTermination(domainBoundary));
			var analyzer = new QuasiStaticLefmAnalyzer(model, algebraicModel, solver, exampleOptions.maxSteps, termination);

			outputOptions.CreateDirectory(exampleOptions, solverOptions);
			string outputDirectory = outputOptions.GetOutputDirectory(exampleOptions, solverOptions);
			var normLogger = new SolutionNormLogger(Path.Combine(outputDirectory, "solution_norm.txt"));
			string header = CreateHeader(exampleOptions, meshOptions, solverOptions, coarseProblemOptions);
			normLogger.ExtraInfo = header;
			analyzer.Results.Add(normLogger);
			analyzer.Logger.ExtraInfo = header;
			solver.Logger.ExtraInfo = header;
			//analyzer.Results.Add(new StructuralFieldWriter(model, outputDirectory));

			var msg = new StringBuilder();
			msg.AppendLine();
			msg.AppendLine("***************************************************************");
			msg.AppendLine(header);
			msg.AppendLine("Starting analysis");
			PrintMsg(msg.ToString());


			analyzer.Analyze();

			string performanceOutputFile = Path.Combine(outputDirectory, "performance.txt");
			//analyzer.Logger.WriteToFile(performanceOutputFile, true);
			solver.Logger.WriteAggregatesToFile(performanceOutputFile, true);
			solver.Logger.WriteToFile(performanceOutputFile, true);

			string convergenceOutputFile = Path.Combine(outputDirectory, "convergence.txt");
			ddmLogger.Header = header;
			ddmLogger.WriteToFile(convergenceOutputFile, true);
		}

		private static (XModel<IXCrackElement>, RectangularDomainBoundary, ComputeNodeTopology) SetupExampleBB4P(
			ExampleBB4POptions exampleOptions, MeshOptions meshOptions, SolverOptions solverOptions)
		{
			FriesExample_7_2_1_Model.heavisideTol = exampleOptions.heavisideTol;
			double[] crackMouthCoords = { exampleOptions.crackFrontX, 0 };
			double[] crackFrontCoords = { exampleOptions.crackFrontX, exampleOptions.crackFrontY };

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

			#region debug
			PrintClusters(nodeTopology);
			PrintNodeNeighbors(nodeTopology);
			#endregion

			FriesExample_7_2_1_Model.CreateGeometryModel(model, meshOptions.numElements, crackMouthCoords, crackFrontCoords);
			var domainBoundary = new RectangularDomainBoundary(
				FriesExample_7_2_1_Model.minCoords, FriesExample_7_2_1_Model.maxCoords);

			return (model, domainBoundary, nodeTopology);
		}

		private static (XModel<IXCrackElement>, RectangularDomainBoundary, ComputeNodeTopology) SetupExampleImpact(
			ExampleImpactOptions exampleOptions, MeshOptions meshOptions, SolverOptions solverOptions)
		{
			FriesExample_7_2_3_Model.heavisideTol = exampleOptions.heavisideTol;
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

			#region debug
			PrintClusters(nodeTopology);
			PrintNodeNeighbors(nodeTopology);
			#endregion

			FriesExample_7_2_3_Model.CreateGeometryModel(model, meshOptions.numElements);
			var domainBoundary = new RectangularDomainBoundary(
				FriesExample_7_2_3_Model.minCoords, FriesExample_7_2_3_Model.maxCoords);

			return (model, domainBoundary, nodeTopology);
		}

		private static (ISolver, IAlgebraicModel, DdmLogger) SetupSolver(MeshOptions meshOptions, SolverOptions solverOptions,
			CoarseProblemOptions coarseProblemOptions, XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
		{
			if (solverOptions.solverType == SolverType.PCG)
			{
				return SetupPcgSolver();
			}
			else if (solverOptions.solverType == SolverType.FETI_DP)
			{
				return SetupFetiDPSolver(meshOptions, solverOptions, coarseProblemOptions, model, nodeTopology);
			}
			else if (solverOptions.solverType == SolverType.PFETI_DP)
			{
				return SetupPFetiDPSolver(meshOptions, solverOptions, coarseProblemOptions, model, nodeTopology);
			}
			else
			{
				throw new NotImplementedException();
			}
		}

		private static (ISolver, IAlgebraicModel, DdmLogger) SetupPcgSolver()
		{
			throw new NotImplementedException();
		}


		private static (ISolver, IAlgebraicModel, DdmLogger) SetupFetiDPSolver(MeshOptions meshOptions, 
			SolverOptions solverOptions, CoarseProblemOptions coarseProblemOptions, 
			XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
		{
			IComputeEnvironment environment = CreateEnvironment(solverOptions, nodeTopology);
			ICornerDofSelection cornerDofs = CreateCornerDofs(meshOptions, solverOptions, environment, model);

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
			solverFactory.CoarseProblemFactory = 
				CreateCoarseProblem(meshOptions, solverOptions, coarseProblemOptions, coarseProblemMatrix);
			solverFactory.EnableLogging = true;
			solverFactory.ExplicitSubdomainMatrices = solverOptions.explicitSchurComplements;
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
			}

			// Create solver
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(model, algebraicModel);
			return (solver, algebraicModel, solver.LoggerDdm);
		}

		private static (ISolver, IAlgebraicModel, DdmLogger) SetupPFetiDPSolver(MeshOptions meshOptions, 
			SolverOptions solverOptions, CoarseProblemOptions coarseProblemOptions, 
			XModel<IXCrackElement> model, ComputeNodeTopology nodeTopology)
		{
			IComputeEnvironment environment = CreateEnvironment(solverOptions, nodeTopology);
			ICornerDofSelection cornerDofs = CreateCornerDofs(meshOptions, solverOptions, environment, model);

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
			solverFactory.CoarseProblemFactory = 
				CreateCoarseProblem(meshOptions, solverOptions, coarseProblemOptions, coarseProblemMatrix);
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
			}

			// Create solver
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(model, algebraicModel);
			return (solver, algebraicModel, solver.LoggerDdm);
		}

		private static IFetiDPCoarseProblemFactory CreateCoarseProblem(MeshOptions meshOptions, SolverOptions solverOptions, 
			CoarseProblemOptions coarseProblemOptions, IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix)
		{
			if (coarseProblemOptions is CoarseProblemOptionsDistributed optionsDistributed)
			{
				var coarseProblemFactory = new FetiDPCoarseProblemDistributed.Factory();
				coarseProblemFactory.UseJacobiPreconditioner = optionsDistributed.jacobiPreconditioner;

				if (optionsDistributed.reorthoPCG)
				{
					var coarseProblemPcgBuilder = new ReorthogonalizedPcg.Builder();
					coarseProblemPcgBuilder.DirectionVectorsRetention = 
						new FixedDirectionVectorsRetention(optionsDistributed.reorthoCacheSize, true);
					coarseProblemPcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(optionsDistributed.pcgMaxIter);
					coarseProblemPcgBuilder.ResidualTolerance = optionsDistributed.pcgTol;
					coarseProblemFactory.CoarseProblemSolver = coarseProblemPcgBuilder.Build();
				}
				else
				{
					var coarseProblemPcgBuilder = new PcgAlgorithm.Builder();
					coarseProblemPcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(optionsDistributed.pcgMaxIter);
					coarseProblemPcgBuilder.ResidualTolerance = optionsDistributed.pcgTol;
					coarseProblemFactory.CoarseProblemSolver = coarseProblemPcgBuilder.Build();
				}
				return coarseProblemFactory;
			}
			else if (coarseProblemOptions is CoarseProblemOptionsGlobal optionsGlobal)
			{
				if (optionsGlobal.numProcesses == 1) // Single machine
				{
					return new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
				}
				else if (optionsGlobal.Democratic)
				{
					solverOptions.environment.GlobalOperationStrategy = new DemocraticGlobalOperationStrategy();
					return new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
				}
				else if (optionsGlobal.MasterProcesses.Length == 1)
				{
					solverOptions.environment.GlobalOperationStrategy = 
						new MasterSlavesGlobalOperationStrategy(optionsGlobal.MasterProcesses[0]);
					return new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
				}
				else
				{
					throw new NotImplementedException();
				}
			}
			else
			{
				throw new NotImplementedException();
			}
		}

		private static ICornerDofSelection CreateCornerDofs(
			MeshOptions meshOptions, SolverOptions solverOptions, IComputeEnvironment environment, XModel<IXCrackElement> model)
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

			CrackFetiDPCornerDofs.strategy = solverOptions.cornerDofStrategy;

			return cornerDofs;
		}

		private static IComputeEnvironment CreateEnvironment(SolverOptions solverOptions, ComputeNodeTopology nodeTopology)
		{
			IComputeEnvironment environment;
			if (solverOptions.environmentChoice == EnvironmentChoice.Serial)
			{
				environment = new SequentialSharedEnvironment(true);
			}
			else if (solverOptions.environmentChoice == EnvironmentChoice.TPL)
			{
				environment = new TplSharedEnvironment(true);
			}
			else
			{
				environment = solverOptions.environment;
			}
			environment.Initialize(nodeTopology);
			return environment;
		}

		private static void PrintClusters(ComputeNodeTopology nodeTopology)
		{
			var msg = new StringBuilder();
			msg.AppendLine();
			msg.AppendLine("*** Subdomains of clusters ***");

			foreach (ComputeNodeCluster cluster in nodeTopology.Clusters.Values)
			{
				msg.Append($"Cluster {cluster.ID} - subdomains ({cluster.Nodes.Count} in total):");
				foreach (ComputeNode subdomain in cluster.Nodes.Values)
				{
					msg.Append($" {subdomain.ID}");
				}
				msg.AppendLine();
			}

			PrintMsg(msg.ToString());
		}

		private static void PrintNodeNeighbors(ComputeNodeTopology nodeTopology)
		{
			var msg = new StringBuilder();
			msg.AppendLine();
			msg.AppendLine("*** Neighbors of subdomains ***");
			foreach (ComputeNode node in nodeTopology.Nodes.Values)
			{
				msg.Append($"subdomain {node.ID} - neighbors ({node.Neighbors.Count} in total): ");
				foreach (int neighbor in node.Neighbors)
				{
					msg.Append($"{neighbor} ");
				}
				msg.AppendLine();
			}

			PrintMsg(msg.ToString());
		}

		private static void PrintMsg(string msg, bool rootOnly = true)
		{
			MPI.Communicator.world.Barrier();
			if (rootOnly)
			{
				if (MPI.Communicator.world.Rank == 0)
				{
					Console.WriteLine(msg);
				}
			}
			else
			{
				Console.WriteLine(msg);
			}
			MPI.Communicator.world.Barrier();
		}
	}
}
