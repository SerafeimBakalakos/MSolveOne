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
		protected readonly IPsmInterfaceProblemMatrix interfaceProblemMatrix;
		protected readonly IDistributedIterativeMethod interfaceProblemSolver;
		protected readonly PsmInterfaceProblemVectors interfaceProblemVectors;
		protected readonly IModel model;
		protected readonly string name;
		protected /*readonly*/ IPsmPreconditioner preconditioner; //TODO: Make this readonly as well.
		protected readonly IBoundaryDofScaling scaling;
		protected readonly ConcurrentDictionary<int, PsmSubdomainDofs> subdomainDofsPsm;
		protected readonly ConcurrentDictionary<int, IPsmSubdomainMatrixManager> subdomainMatricesPsm;
		protected readonly SubdomainTopology subdomainTopology;
		protected readonly ConcurrentDictionary<int, PsmSubdomainVectors> subdomainVectors;

		protected DistributedOverlappingIndexer boundaryDofIndexer; //TODOMPI: Perhaps this should be accessed from DofSeparator

		protected PsmSolver(IComputeEnvironment environment, IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel, 
			IPsmSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory, 
			bool explicitSubdomainMatrices, IPsmPreconditioner preconditioner,
			IDistributedIterativeMethod interfaceProblemSolver, bool isHomogeneous, string name = "PSM Solver")
		{
			this.name = name;
			this.environment = environment;
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.subdomainTopology = algebraicModel.SubdomainTopology;
			this.LinearSystem = algebraicModel.LinearSystem;
			this.preconditioner = preconditioner;
			this.interfaceProblemSolver = interfaceProblemSolver;

			this.subdomainDofsPsm = new ConcurrentDictionary<int, PsmSubdomainDofs>();
			this.subdomainMatricesPsm = new ConcurrentDictionary<int, IPsmSubdomainMatrixManager>();
			this.subdomainVectors = new ConcurrentDictionary<int, PsmSubdomainVectors>();
			environment.DoPerNode(subdomainID =>
			{
				SubdomainLinearSystem<TMatrix> linearSystem = algebraicModel.SubdomainLinearSystems[subdomainID];
				var dofs = new PsmSubdomainDofs(model.AllDofs, linearSystem, false);
				IPsmSubdomainMatrixManager matrices = matrixManagerFactory.CreateMatrixManager(linearSystem, dofs);
				var vectors = new PsmSubdomainVectors(linearSystem, dofs, matrices);

				subdomainDofsPsm[subdomainID] = dofs;
				subdomainMatricesPsm[subdomainID] = matrices;
				subdomainVectors[subdomainID] = vectors;
			});

			if (isHomogeneous)
			{
				this.scaling = new HomogeneousScaling(environment, s => subdomainDofsPsm[s]);
			}
			else
			{
				this.scaling = new HeterogeneousScaling(
					environment, s => algebraicModel.SubdomainLinearSystems[s], s => subdomainDofsPsm[s]);
			}

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

		public virtual void HandleMatrixWillBeSet()
		{
		}

		public virtual void PreventFromOverwrittingSystemMatrices() {}

		public virtual void Solve() 
		{
			// Prepare subdomain-level dofs and matrices
			environment.DoPerNode(subdomainID =>
			{
				//TODO: These should only happen if the connectivity of the subdomain changes. 
				subdomainDofsPsm[subdomainID].SeparateFreeDofsIntoBoundaryAndInternal();
				subdomainMatricesPsm[subdomainID].ReorderInternalDofs();

				//TODO: These should happen if the connectivity or stiffness of the subdomain changes
				subdomainMatricesPsm[subdomainID].ExtractKiiKbbKib();
				subdomainMatricesPsm[subdomainID].InvertKii();
			});

			// Intersubdomain dofs
			this.boundaryDofIndexer = subdomainTopology.CreateDistributedVectorIndexer(
				s => subdomainDofsPsm[s].DofOrderingBoundary);

			// Calculating scaling coefficients
			scaling.CalcScalingMatrices(boundaryDofIndexer);

			// Prepare subdomain-level vectors
			environment.DoPerNode(subdomainID =>
			{
				subdomainVectors[subdomainID].Clear();
				subdomainVectors[subdomainID].ExtractBoundaryInternalRhsVectors(
					fb => scaling.ScaleBoundaryRhsVector(subdomainID, fb));
			});

			// Prepare and solve the interface problem
			interfaceProblemMatrix.Calculate(boundaryDofIndexer);
			CalcPreconditioner();
			SolveInterfaceProblem();

			// Find the solution at all free dofs
			environment.DoPerNode(subdomainID =>
			{
				Vector subdomainBoundarySolution = interfaceProblemVectors.InterfaceProblemSolution.LocalVectors[subdomainID];
				subdomainVectors[subdomainID].CalcSubdomainFreeSolution(subdomainBoundarySolution);
			});

			Logger.IncrementAnalysisStep();
		}

		protected virtual void CalcPreconditioner()
		{
			preconditioner.Calculate(environment, boundaryDofIndexer, interfaceProblemMatrix);
		}

		protected void SolveInterfaceProblem()
		{
			interfaceProblemVectors.Clear();
			interfaceProblemVectors.CalcInterfaceRhsVector(boundaryDofIndexer);
			bool initalGuessIsZero = !StartIterativeSolverFromPreviousSolution;
			if (!StartIterativeSolverFromPreviousSolution)
			{
				interfaceProblemVectors.InterfaceProblemSolution = new DistributedOverlappingVector(boundaryDofIndexer);
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
			protected readonly IComputeEnvironment environment;

			public Factory(IComputeEnvironment environment, IPsmSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory)
			{
				DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
				ExplicitSubdomainMatrices = false;

				//TODO: perhaps use a custom convergence check like in FETI
				var pcgBuilder = new PcgAlgorithm.Builder();
				pcgBuilder.ResidualTolerance = 1E-6;
				pcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(100);
				InterfaceProblemSolver = pcgBuilder.Build();
				IsHomogeneousProblem = true;

				PsmMatricesFactory = matrixManagerFactory; //new PsmSubdomainMatrixManagerSymmetricCSparse.Factory();
				Preconditioner = new PsmPreconditionerIdentity();
				this.environment = environment;
			}

			public IDofOrderer DofOrderer { get; set; }

			public bool ExplicitSubdomainMatrices { get; set; }

			public IDistributedIterativeMethod InterfaceProblemSolver { get; set; }

			public bool IsHomogeneousProblem { get; set; }

			public IPsmSubdomainMatrixManagerFactory<TMatrix> PsmMatricesFactory { get; }

			public IPsmPreconditioner Preconditioner { get; set; }

			public DistributedAlgebraicModel<TMatrix> BuildAlgebraicModel(IModel model)
				=> new DistributedAlgebraicModel<TMatrix>(environment, model, DofOrderer, PsmMatricesFactory.CreateAssembler());

			public virtual PsmSolver<TMatrix> BuildSolver(IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel)
			{
				return new PsmSolver<TMatrix>(environment, model, algebraicModel, PsmMatricesFactory,
					ExplicitSubdomainMatrices, Preconditioner, InterfaceProblemSolver, IsHomogeneousProblem);
			}
		}
	}
}
