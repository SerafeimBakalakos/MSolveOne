using System;
using System.Collections.Generic;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Iterative.Termination;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;
using MGroup.Solvers.DDM.Prototypes.StrategyEnums;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
	public class PsmSolver : ISolver
	{
		protected readonly IModel model;
		protected readonly DistributedAlgebraicModel<Matrix> algebraicModel;
		protected readonly IPsmInterfaceProblem interfaceProblem;
		protected readonly PcgAlgorithm iterativeSolver;
		protected readonly PsmSubdomainDofs psmDofs;
		protected readonly IPsmScaling scaling;
		protected readonly PsmSubdomainStiffnesses psmStiffnesses;
		protected readonly PsmSubdomainVectors vectors;
		protected readonly DenseMatrixAssembler assembler = new DenseMatrixAssembler();

		public PsmSolver(IModel model, DistributedAlgebraicModel<Matrix> algebraicModel, bool homogeneousProblem, 
			double pcgTolerance, int maxPcgIterations, PsmInterfaceProblem interfaceProblem)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.psmDofs = new PsmSubdomainDofs(model, algebraicModel);
			this.psmStiffnesses = new PsmSubdomainStiffnesses(model, psmDofs);
			this.vectors = new PsmSubdomainVectors(model, psmDofs, psmStiffnesses);

			// Distributed vs global
			if (interfaceProblem == PsmInterfaceProblem.Original)
			{
				this.interfaceProblem = new PsmInterfaceProblemOriginal(model, psmDofs);
			}
			else if (interfaceProblem == PsmInterfaceProblem.Distributed)
			{
				this.interfaceProblem = new PsmInterfaceProblemDistributed(model, psmDofs);
			}

			// Scaling
			if (homogeneousProblem)
			{
				scaling = new HomogeneousScaling(model, algebraicModel, psmDofs);
			}
			else
			{
				scaling = new HeterogeneousScaling(model);
			}

			// Iterative solver
			var pcgBuilder = new PcgAlgorithm.Builder();
			pcgBuilder.ResidualTolerance = pcgTolerance;
			pcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(maxPcgIterations);
			this.iterativeSolver = pcgBuilder.Build();
		}

		public IGlobalLinearSystem LinearSystem => algebraicModel.LinearSystem;

		public ISolverLogger Logger => throw new NotImplementedException();

		public string Name => throw new NotImplementedException();

		public IterativeStatistics IterativeSolverStats { get; set; }

		public void HandleMatrixWillBeSet()
		{
			
		}

		public virtual void Initialize()
		{
			
		}

		public void PreventFromOverwrittingSystemMatrices()
		{
		}

		public virtual void Solve()
		{
			// Prepare dofs and mappings
			psmDofs.FindDofs();
			interfaceProblem.FindDofs();

			// Prepare interface problem
			DistributedOverlappingMatrix<Matrix> Kff = algebraicModel.LinearSystem.Matrix;
			DistributedOverlappingVector Ff = algebraicModel.LinearSystem.RhsVector;
			psmStiffnesses.CalcAllMatrices(s => Kff.LocalMatrices[s]);
			scaling.CalcScalingMatrices(s => Kff.LocalMatrices[s]);
			vectors.CalcAllRhsVectors(s => Ff.LocalVectors[s], scaling.ScaleRhsVector);
			BlockMatrix Sbbe = psmStiffnesses.Sbbe;
			BlockVector FbeCondensed = vectors.FbeCondensed;
			BlockVector Ube = FbeCondensed.CreateZeroVectorWithSameFormat();

			// Calculate the preconditioner
			IPreconditioner preconditioner = CalcPreconditioner();

			// Solve interface problem
			this.IterativeSolverStats = interfaceProblem.Solve(iterativeSolver, preconditioner, Sbbe, FbeCondensed, Ube);

			// Find displacements at free dofs
			CalcFreeDisplacements(Ube);
		}

		protected virtual IPreconditioner CalcPreconditioner() => new IdentityPreconditioner();

		protected void CalcFreeDisplacements(BlockVector Ube)
		{
			DistributedOverlappingVector Uf = algebraicModel.LinearSystem.Solution;
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				Vector Ub = Ube.Blocks[s];
				vectors.CalcFreeDisplacements(s, Ub);
				Uf.LocalVectors[s] = vectors.Uf[s];
			}
		}

		public class Factory
		{
			private readonly bool homogeneousProblem;
			private readonly double pcgTolerance;
			private readonly int maxPcgIterations;
			private readonly PsmInterfaceProblem interfaceProblem;

			public Factory(bool homogeneousProblem, double pcgTolerance, int maxPcgIterations,
				PsmInterfaceProblem interfaceProblem) 
			{
				this.homogeneousProblem = homogeneousProblem;
				this.pcgTolerance = pcgTolerance;
				this.maxPcgIterations = maxPcgIterations;
				this.interfaceProblem = interfaceProblem;
			}

			public IDofOrderer DofOrderer { get; set; }
				= new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());

			public bool IsMatrixPositiveDefinite { get; set; } = true;

			public PsmSolver BuildSolver(IModel model, DistributedAlgebraicModel<Matrix> algebraicModel)
				=> new PsmSolver(model, algebraicModel, homogeneousProblem, pcgTolerance, maxPcgIterations, interfaceProblem);

			public DistributedAlgebraicModel<Matrix> BuildAlgebraicModel(IComputeEnvironment environment, IModel model)
				=> new DistributedAlgebraicModel<Matrix>(environment, model, DofOrderer, new DenseMatrixAssembler());
		}
	}
}
