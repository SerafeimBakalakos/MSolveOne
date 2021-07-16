using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;

using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.IterativeMethods;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.Termination;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PSM.Dofs;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.Preconditioning;
using MGroup.Solvers.DDM.PSM.Scaling;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.DDM.PSM.Vectors;
using MGroup.Solvers.DDM.StiffnessMatrices;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.Solvers.Logging;

namespace MGroup.Solvers.DDM.Psm
{
	public class PsmSolver<TMatrix> : ISolver
		where TMatrix : class, IMatrix
	{
		protected readonly DistributedAlgebraicModel<TMatrix> algebraicModel;
		protected readonly IComputeEnvironment environment;
		protected readonly PsmInterfaceProblemDofs interfaceProblemDofs;
		protected readonly IPsmInterfaceProblemMatrix interfaceProblemMatrix;
		protected readonly IDistributedIterativeMethod interfaceProblemSolver;
		protected readonly PsmInterfaceProblemVectors interfaceProblemVectors;
		protected readonly IModel model;
		protected readonly string name;
		protected readonly IPsmPreconditioner preconditioner;
		protected readonly IBoundaryDofScaling stiffnessDistribution;
		protected readonly ConcurrentDictionary<int, PsmSubdomainDofs> subdomainDofsPsm;
		protected readonly ConcurrentDictionary<int, IPsmSubdomainMatrixManager> subdomainMatricesPsm;
		protected readonly SubdomainTopology subdomainTopology;
		protected readonly ConcurrentDictionary<int, PsmSubdomainVectors> subdomainVectors;

		private DistributedOverlappingIndexer indexer; //TODOMPI: Perhaps this should be accessed from DofSeparator

		protected PsmSolver(IComputeEnvironment environment, IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel, 
			SubdomainTopology subdomainTopology, IPsmSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory, 
			bool explicitSubdomainMatrices, IPsmPreconditioner preconditioner,
			IDistributedIterativeMethod interfaceProblemSolver, bool isHomogeneous, string name = "PSM Solver")
		{
			this.name = name;
			this.environment = environment;
			this.model = model;
			this.subdomainTopology = subdomainTopology;
			this.algebraicModel = algebraicModel;
			this.LinearSystem = algebraicModel.LinearSystem;
			this.preconditioner = preconditioner;
			this.interfaceProblemSolver = interfaceProblemSolver;

			this.subdomainDofsPsm = new ConcurrentDictionary<int, PsmSubdomainDofs>();
			this.subdomainMatricesPsm = new ConcurrentDictionary<int, IPsmSubdomainMatrixManager>();
			this.subdomainVectors = new ConcurrentDictionary<int, PsmSubdomainVectors>();
			environment.DoPerNode(subdomainID =>
			{
				SubdomainLinearSystem<TMatrix> linearSystem = algebraicModel.SubdomainLinearSystems[subdomainID];
				var dofs = new PsmSubdomainDofs(linearSystem, false);
				IPsmSubdomainMatrixManager matrices = matrixManagerFactory.CreateMatrixManager(linearSystem, dofs);
				var vectors = new PsmSubdomainVectors(linearSystem, dofs, matrices);

				subdomainDofsPsm[subdomainID] = dofs;
				subdomainMatricesPsm[subdomainID] = matrices;
				subdomainVectors[subdomainID] = vectors;
			});

			if (isHomogeneous)
			{
				this.stiffnessDistribution = new HomogeneousScaling(environment, s => subdomainDofsPsm[s]);
			}
			else
			{
				this.stiffnessDistribution = new HeterogeneousScaling(
					environment, s => algebraicModel.SubdomainLinearSystems[s], s => subdomainDofsPsm[s]);
			}

			this.interfaceProblemDofs = new PsmInterfaceProblemDofs(environment, model, subdomainTopology, 
				s => algebraicModel.SubdomainLinearSystems[s], s => subdomainDofsPsm[s]);
			this.interfaceProblemVectors = new PsmInterfaceProblemVectors(environment, subdomainVectors);
			if (explicitSubdomainMatrices)
			{
				this.interfaceProblemMatrix = new PsmInterfaceProblemMatrixExplicit(environment, s => subdomainMatricesPsm[s]);
			}
			else
			{
				this.interfaceProblemMatrix = new PsmInterfaceProblemMatrixImplicit(environment, 
					s => subdomainDofsPsm[s], s => subdomainMatricesPsm[s]);
			}

			Logger = new SolverLogger(name);
		}

		public IterativeStatistics InterfaceProblemSolutionStats { get; private set; }

		public IGlobalLinearSystem LinearSystem { get; }

		public ISolverLogger Logger { get; }

		public string Name => name;

		public bool StartIterativeSolverFromPreviousSolution { get; set; } = false;

		public virtual Dictionary<int, SparseVector> DistributeNodalLoads(Table<INode, IDofType, double> nodalLoads)
			=> stiffnessDistribution.DistributeNodalLoads(nodalLoads);

		public virtual void HandleMatrixWillBeSet()
		{
		}

		public virtual void Initialize() //TODOMPI: Restructure this. The analyzers now call only solver.Solve()
		{
			// Reordering the internal dofs is not done here, since subdomain Kff must be built first. 
			environment.DoPerNode(subdomainID =>
			{
				subdomainDofsPsm[subdomainID].SeparateFreeDofsIntoBoundaryAndInternal();
			}); 
			interfaceProblemDofs.FindCommonDofsBetweenSubdomains();
			this.indexer = interfaceProblemDofs.CreateDistributedVectorIndexer();

			//TODOMPI: What should I log here? And where? There is not a central place for logs.
			//Logger.LogNumDofs("Global boundary dofs", dofSeparatorPsm.GetNumBoundaryDofsCluster(clusterID));
		}

		public virtual void PreventFromOverwrittingSystemMatrices() {}

		public virtual void Solve() 
		{
			Initialize();
			Action<int> calcSubdomainMatrices = subdomainID =>
			{
				//TODO: This should only happen if the connectivity of the subdomain changes. 
				subdomainMatricesPsm[subdomainID].ReorderInternalDofs();

				//TODO: These should happen if the connectivity or stiffness of the subdomain changes
				subdomainMatricesPsm[subdomainID].ExtractKiiKbbKib();
				subdomainMatricesPsm[subdomainID].InvertKii();

				subdomainVectors[subdomainID].Clear();
				subdomainVectors[subdomainID].ExtractInternalRhsVector();
			};
			environment.DoPerNode(calcSubdomainMatrices);

			interfaceProblemMatrix.Calculate(indexer);
			preconditioner.Calculate(environment, indexer, interfaceProblemMatrix);

			SolveInterfaceProblem();

			environment.DoPerNode(subdomainID =>
			{
				Vector subdomainBoundarySolution = interfaceProblemVectors.InterfaceProblemSolution.LocalVectors[subdomainID];
				subdomainVectors[subdomainID].CalcSubdomainFreeSolution(subdomainBoundarySolution);
			});

			Logger.IncrementAnalysisStep();
		}

		protected void SolveInterfaceProblem()
		{
			interfaceProblemVectors.Clear();
			interfaceProblemVectors.CalcInterfaceRhsVector(indexer);
			bool initalGuessIsZero = !StartIterativeSolverFromPreviousSolution;
			if (!StartIterativeSolverFromPreviousSolution)
			{
				interfaceProblemVectors.InterfaceProblemSolution = new DistributedOverlappingVector(environment, indexer);
			}
			IterativeStatistics stats = interfaceProblemSolver.Solve(
				interfaceProblemMatrix.Matrix, preconditioner.Preconditioner, interfaceProblemVectors.InterfaceProblemRhs,
				interfaceProblemVectors.InterfaceProblemSolution, initalGuessIsZero);
			InterfaceProblemSolutionStats = stats;
			Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
			Debug.WriteLine("Iterations for boundary problem = " + stats.NumIterationsRequired);
		}

		public class Factory
		{
			private readonly IComputeEnvironment environment;

			public Factory(IComputeEnvironment environment, IPsmSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory)
			{
				DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering(), true);
				ExplicitSubdomainMatrices = false;

				//TODO: perhaps use a custom convergence check like in FETI
				var pcgBuilder = new PcgAlgorithm.Builder();
				pcgBuilder.ResidualTolerance = 1E-6;
				pcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(100);
				InterfaceProblemSolver = pcgBuilder.Build();
				IsHomogeneousProblem = true;

				MatrixManagerFactory = matrixManagerFactory; //new PsmSubdomainMatrixManagerSymmetricCSparse.Factory();
				Preconditioner = new PsmPreconditionerIdentity();
				this.environment = environment;
			}

			public IDofOrderer DofOrderer { get; set; }

			public bool ExplicitSubdomainMatrices { get; set; }

			public IDistributedIterativeMethod InterfaceProblemSolver { get; set; }

			public bool IsHomogeneousProblem { get; set; }

			public IPsmSubdomainMatrixManagerFactory<TMatrix> MatrixManagerFactory { get; set; }

			public IPsmPreconditioner Preconditioner { get; set; }

			public DistributedAlgebraicModel<TMatrix> BuildAlgebraicModel(IModel model)
				=> new DistributedAlgebraicModel<TMatrix>(model, DofOrderer, MatrixManagerFactory.CreateAssembler());

			public PsmSolver<TMatrix> BuildSolver(IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel, 
				SubdomainTopology subdomainTopology)
			{
				return new PsmSolver<TMatrix>(environment, model, algebraicModel, subdomainTopology, 
					MatrixManagerFactory, ExplicitSubdomainMatrices, Preconditioner, InterfaceProblemSolver, IsHomogeneousProblem);
			}
		}
	}
}
