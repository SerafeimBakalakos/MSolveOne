using System;
using MGroup.Environments;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.Prototypes.FetiDP;
using MGroup.Solvers.DDM.Prototypes.PSM;
using MGroup.Solvers.DDM.Prototypes.StrategyEnums;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	public class PFetiDPSolver : PsmSolver
	{
		private readonly IFetiDPCoarseProblem coarseProblem;
		private readonly FetiDPSubdomainDofs fetiDPDofs;
		private readonly FetiDPSubdomainStiffnesses fetiDPStiffnesses;
		private readonly IInterfaceCoarseProblemBridge interfaceCoarseProblemBridge;
		private readonly PFetiDPSubdomainDofs pfetiDPDofs;
		private readonly IPFetiDPScaling pfetiDPScaling;
		private readonly IPFetiDPPreconditioner preconditioner;

		public PFetiDPSolver(IModel model, DistributedAlgebraicModel<Matrix> algebraicModel, DDM.FetiDP.Dofs.ICornerDofSelection cornerDofs, 
			bool homogeneousProblem, double pcgTolerance, int maxPcgIterations, PsmInterfaceProblem interfaceProblemChoice, 
			FetiDPCoarseProblem coarseProblemChoice, PFetiDPScaling scalingChoice, PFetiDPPreconditioner preconditionerChoice) 
			: base(model, algebraicModel, homogeneousProblem, pcgTolerance, maxPcgIterations, interfaceProblemChoice)
		{
			this.fetiDPDofs = new FetiDPSubdomainDofs(model, algebraicModel, cornerDofs);
			this.pfetiDPDofs = new PFetiDPSubdomainDofs(model, psmDofs, fetiDPDofs);
			this.fetiDPStiffnesses = new FetiDPSubdomainStiffnesses(model, fetiDPDofs);

			if (coarseProblemChoice == FetiDPCoarseProblem.Original)
			{
				this.coarseProblem = new FetiDPCoarseProblemOriginal(model, fetiDPDofs, fetiDPStiffnesses);
			}
			else if (coarseProblemChoice == FetiDPCoarseProblem.Distributed)
			{
				this.coarseProblem = new FetiDPCoarseProblemDistributed(model, fetiDPDofs, fetiDPStiffnesses, pcgTolerance);
			}
			else if (coarseProblemChoice == FetiDPCoarseProblem.DistributedJacobi)
			{
				this.coarseProblem = new FetiDPCoarseProblemDistributedJacobi(model, fetiDPDofs, fetiDPStiffnesses, pcgTolerance);
			}
			else if (coarseProblemChoice == FetiDPCoarseProblem.DistributedReortho)
			{
				this.coarseProblem = new FetiDPCoarseProblemDistributedReortho(model, fetiDPDofs, fetiDPStiffnesses, pcgTolerance);
			}
			else if (coarseProblemChoice == FetiDPCoarseProblem.DistributedJacobiReortho)
			{
				this.coarseProblem = new FetiDPCoarseProblemDistributedJacobiReortho(model, fetiDPDofs, fetiDPStiffnesses, pcgTolerance);
			}
			else
			{
				throw new NotImplementedException();
			}

			if (interfaceProblemChoice == PsmInterfaceProblem.Original)
			{
				this.interfaceCoarseProblemBridge = new InterfaceCoarseProblemBridgeOriginal();
			}
			else if (interfaceProblemChoice == PsmInterfaceProblem.Distributed)
			{
				this.interfaceCoarseProblemBridge = new InterfaceCoarseProblemBridgeDistributed();
			}
			else
			{
				throw new NotImplementedException();
			}

			if (scalingChoice == PFetiDPScaling.HomogeneousOriginal)
			{
				this.pfetiDPScaling = new PFetiDPHomogeneousScalingOriginal(model);
			}
			else if (scalingChoice == PFetiDPScaling.HomogeneousModified)
			{
				this.pfetiDPScaling = new PFetiDPHomogeneousScalingModified(model);
			}
			else if (scalingChoice == PFetiDPScaling.HomogeneousDistributed)
			{
				this.pfetiDPScaling = new PFetiDPHomogeneousScalingDistributed(model);
			}
			else
			{
				throw new NotImplementedException();
			}

			if (preconditionerChoice == PFetiDPPreconditioner.OriginalMonolithic)
			{
				this.preconditioner = new PFetiDPPreconditionerOriginal();
			}
			else if (preconditionerChoice == PFetiDPPreconditioner.OriginalDistributive)
			{
				this.preconditioner = new PFetiDPPreconditionerOriginalDistributive();
			}
			else if (preconditionerChoice == PFetiDPPreconditioner.DistributedInterfaceOriginalCoarse)
			{
				this.preconditioner = new PFetiDPPreconditionerDistributedInterface();
			}
			else if (preconditionerChoice == PFetiDPPreconditioner.DistributedAll)
			{
				this.preconditioner = new PFetiDPPreconditionerDistributedAll();
			}
			else
			{
				throw new NotImplementedException();
			}
		}

		internal IFetiDPCoarseProblem CoarseProblem => coarseProblem;
		internal FetiDPSubdomainDofs FetiDPDofs => fetiDPDofs;
		internal FetiDPSubdomainStiffnesses FetiDPStiffnesses => fetiDPStiffnesses;
		internal IInterfaceCoarseProblemBridge InterfaceCoarseProblemBridge => interfaceCoarseProblemBridge;
		internal IPsmInterfaceProblem InterfaceProblem => interfaceProblem;
		internal PFetiDPSubdomainDofs PFetiDPDofs => pfetiDPDofs;
		internal IPFetiDPPreconditioner Preconditioner => preconditioner;
		internal PsmSubdomainDofs PsmDofs => psmDofs;
		internal IPFetiDPScaling Scaling => pfetiDPScaling;

		protected override IPreconditioner CalcPreconditioner()
		{
			// Initialize components
			InterfaceCoarseProblemBridge.InitializeComponents(this);
			Scaling.InitializeComponents(this);
			Preconditioner.InitializeComponents(this);

			// Prepare dofs and mappings
			fetiDPDofs.FindDofs();
			coarseProblem.FindDofs();
			pfetiDPDofs.MapPsmFetiDPDofs();
			interfaceCoarseProblemBridge.LinkInterfaceCoarseProblems();
			pfetiDPScaling.Initialize();

			// Prepare FetiDP matrices
			DistributedOverlappingMatrix<Matrix> Kff = algebraicModel.LinearSystem.Matrix;
			fetiDPStiffnesses.CalcAllMatrices(s => Kff.LocalMatrices[s]);
			coarseProblem.Prepare();

			// Calculate preconditioner
			preconditioner.Calculate();
			return preconditioner;
		}

		public class Factory
		{
			private readonly bool homogeneousProblem;
			private readonly double pcgTolerance;
			private readonly int maxPcgIterations;
			private readonly PsmInterfaceProblem interfaceProblemChoice;
			private readonly FetiDPCoarseProblem coarseProblemChoice;
			private readonly PFetiDPScaling scalingChoice;
			private readonly PFetiDPPreconditioner preconditionerChoice;

			public Factory(bool homogeneousProblem, double pcgTolerance, int maxPcgIterations, 
				PsmInterfaceProblem interfaceProblemChoice, FetiDPCoarseProblem coarseProblemChoice, PFetiDPScaling scalingChoice, 
				PFetiDPPreconditioner preconditionerChoice)
			{
				this.homogeneousProblem = homogeneousProblem;
				this.pcgTolerance = pcgTolerance;
				this.maxPcgIterations = maxPcgIterations;
				this.interfaceProblemChoice = interfaceProblemChoice;
				this.coarseProblemChoice = coarseProblemChoice;
				this.scalingChoice = scalingChoice;
				this.preconditionerChoice = preconditionerChoice;
			}

			public IDofOrderer DofOrderer { get; set; }
				= new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());

			public bool IsMatrixPositiveDefinite { get; set; } = true;

			public PFetiDPSolver BuildSolver(IModel model, DDM.FetiDP.Dofs.ICornerDofSelection cornerDofs, 
				DistributedAlgebraicModel<Matrix> algebraicModel)
				=> new PFetiDPSolver(model, algebraicModel, cornerDofs, homogeneousProblem, pcgTolerance, maxPcgIterations, 
					interfaceProblemChoice, coarseProblemChoice, scalingChoice, preconditionerChoice);

			public DistributedAlgebraicModel<Matrix> BuildAlgebraicModel(IComputeEnvironment environment, IModel model)
				=> new DistributedAlgebraicModel<Matrix>(environment, model, DofOrderer, new DenseMatrixAssembler());
		}
	}
}
