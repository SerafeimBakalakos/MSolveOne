using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.Termination;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.IterativeMethods;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.Solvers.DDM.PSM.Dofs;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.Preconditioning;
using MGroup.Solvers.DDM.PSM.Scaling;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.DDM.PSM.Vectors;
using MGroup.Solvers.DDM.StiffnessMatrices;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.Logging;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.MSolve.DataStructures;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.Solvers.AlgebraicModel;

namespace MGroup.Solvers.DDM.Psm
{
	public class PsmSolver<TMatrix> : ISolver
		where TMatrix : class, IMatrix
	{
		protected readonly DistributedAlgebraicModel<TMatrix> algebraicModel;
		protected readonly IDofOrderer dofOrderer;
		protected readonly PsmDofManager dofManagerPsm;
		protected readonly IComputeEnvironment environment;
		protected readonly IPsmInterfaceProblemMatrix interfaceProblemMatrix;
		protected readonly IDistributedIterativeMethod interfaceProblemSolver;
		protected readonly PsmInterfaceProblemVectors interfaceProblemVectors;
		protected readonly ConcurrentDictionary<int, ISubdomainMatrixManager> matrixManagersBasic;
		protected readonly ConcurrentDictionary<int, IPsmSubdomainMatrixManagerGeneric<TMatrix>> matrixManagersPsmGeneric; //TODO: refactor this
		protected readonly IModel model;
		protected readonly string name;
		protected readonly IPsmPreconditioner preconditioner;
		protected readonly IBoundaryDofScaling stiffnessDistribution;
		protected readonly SubdomainTopology subdomainTopology;
		protected readonly Dictionary<int, PsmSubdomainVectors> subdomainVectors;

		private DistributedOverlappingIndexer indexer; //TODOMPI: Perhaps this should be accessed from DofSeparator

		protected PsmSolver(IComputeEnvironment environment, IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel, 
			SubdomainTopology subdomainTopology, IDofOrderer dofOrderer, IPsmSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory, 
			bool explicitSubdomainMatrices, IPsmPreconditioner preconditioner,
			IDistributedIterativeMethod interfaceProblemSolver, bool isHomogeneous, string name = "PSM Solver")
		{
			this.name = name;
			this.environment = environment;
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.subdomainTopology = subdomainTopology;
			this.dofOrderer = dofOrderer;
			this.preconditioner = preconditioner;
			this.interfaceProblemSolver = interfaceProblemSolver;

			this.dofManagerPsm = new PsmDofManager(environment, model, subdomainTopology, 
				s => algebraicModel.DofOrdering.SubdomainDofOrderings[s], false);

			matrixManagersPsmGeneric = new ConcurrentDictionary<int, IPsmSubdomainMatrixManagerGeneric<TMatrix>>();
			var matrixManagersPsm = new ConcurrentDictionary<int, IPsmSubdomainMatrixManager>();
			environment.DoPerNode(subdomainID =>
			{
				ISubdomain subdomain = model.GetSubdomain(subdomainID);
				PsmSubdomainDofs subdomainDofs = dofManagerPsm.GetSubdomainDofs(subdomainID);
				IPsmSubdomainMatrixManagerGeneric<TMatrix> matrixManagerPsm = 
					matrixManagerFactory.CreateMatrixManager(subdomainDofs);
				matrixManagersPsmGeneric[subdomainID] = matrixManagerPsm;
				matrixManagersPsm[subdomainID] = matrixManagerPsm;
			});

			if (explicitSubdomainMatrices)
			{
				this.interfaceProblemMatrix = new PsmInterfaceProblemMatrixExplicit(environment, matrixManagersPsm);
			}
			else
			{
				this.interfaceProblemMatrix = new PsmInterfaceProblemMatrixImplicit(environment, dofManagerPsm, matrixManagersPsm);
			}

			if (isHomogeneous)
			{
				this.stiffnessDistribution = new HomogeneousScaling(environment, model, dofManagerPsm);
			}
			else
			{
				this.stiffnessDistribution = new HeterogeneousScaling(
					environment, model, dofManagerPsm, matrixManagersBasic);
			}

			LinearSystem = null;
			//LinearSystems = environment.CreateDictionaryPerNode(
			//	subdomainID => matrixManagersBasic[subdomainID].LinearSystem);

			this.subdomainVectors = environment.CreateDictionaryPerNode(subdomainID => new PsmSubdomainVectors(
					dofManagerPsm.GetSubdomainDofs(subdomainID), matrixManagersPsm[subdomainID]));

			this.interfaceProblemVectors = new PsmInterfaceProblemVectors(
				environment, algebraicModel.LinearSystem, subdomainVectors);

			Logger = new SolverLogger(name);
		}

		//public IReadOnlyDictionary<int, ILinearSystem> LinearSystems { get; }
		public IGlobalLinearSystem LinearSystem { get; }

		public ISolverLogger Logger { get; }

		public string Name => name;

		public bool StartIterativeSolverFromPreviousSolution { get; set; } = false;

		public virtual Dictionary<int, SparseVector> DistributeNodalLoads(Table<INode, IDofType, double> nodalLoads)
			=> stiffnessDistribution.DistributeNodalLoads(nodalLoads);

		public virtual void HandleMatrixWillBeSet()
		{
		}

		public virtual void Initialize()
		{
			// Reordering the internal dofs is not done here, since subdomain Kff must be built first. 
			environment.DoPerNode(subdomainID =>
			{
				ISubdomainFreeDofOrdering dofOrdering = algebraicModel.DofOrdering.SubdomainDofOrderings[subdomainID];
				dofManagerPsm.GetSubdomainDofs(subdomainID).SeparateFreeDofsIntoBoundaryAndInternal();
			}); 
			dofManagerPsm.FindCommonDofsBetweenSubdomains();
			this.indexer = dofManagerPsm.CreateDistributedVectorIndexer();

			//TODOMPI: What should I log here? And where? There is not a central place for logs.
			//Logger.LogNumDofs("Global boundary dofs", dofSeparatorPsm.GetNumBoundaryDofsCluster(clusterID));
		}

		public virtual void PreventFromOverwrittingSystemMatrices() {}

		public virtual void Solve()
		{
			Action<int> calcSubdomainMatrices = subdomainID =>
			{
				TMatrix Kff = algebraicModel.LinearSystem.Matrix.LocalMatrices[subdomainID];
				Vector Ff = algebraicModel.LinearSystem.RhsVector.LocalVectors[subdomainID];

				//TODO: This should only happen if the connectivity of the subdomain changes. 
				matrixManagersPsmGeneric[subdomainID].ReorderInternalDofs(Kff);

				//TODO: These should happen if the connectivity or stiffness of the subdomain changes
				matrixManagersPsmGeneric[subdomainID].ExtractKiiKbbKib(Kff);
				matrixManagersPsmGeneric[subdomainID].InvertKii();

				subdomainVectors[subdomainID].Clear();
				subdomainVectors[subdomainID].ExtractInternalRhsVector(Ff);
			};
			environment.DoPerNode(calcSubdomainMatrices);

			interfaceProblemMatrix.Calculate(indexer);
			preconditioner.Calculate(environment, indexer, interfaceProblemMatrix);

			SolveInterfaceProblem();

			environment.DoPerNode(subdomainID =>
			{
				Vector subdomainBoundarySolution = interfaceProblemVectors.InterfaceProblemSolution.LocalVectors[subdomainID];
				Vector uf = subdomainVectors[subdomainID].CalcSubdomainFreeSolution(subdomainBoundarySolution);
				algebraicModel.LinearSystem.Solution.LocalVectors[subdomainID] = uf;
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
				return new PsmSolver<TMatrix>(environment, model, algebraicModel, subdomainTopology, DofOrderer, 
					MatrixManagerFactory, ExplicitSubdomainMatrices, Preconditioner, InterfaceProblemSolver, IsHomogeneousProblem);
			}
		}
	}
}
