using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.Output;
using MGroup.Solvers.DDM.PFetiDP.Dofs;
using MGroup.Solvers.DDM.PFetiDP.Preconditioner;
using MGroup.Solvers.DDM.Psm;
using MGroup.Solvers.DDM.PSM;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.Preconditioning;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;

namespace MGroup.Solvers.DDM.PFetiDP
{
	public class PFetiDPSolver<TMatrix> : PsmSolver<TMatrix>
		where TMatrix : class, IMatrix
	{
		private readonly ICornerDofSelection cornerDofs;
		private readonly IFetiDPCoarseProblem coarseProblemFetiDP;
		private readonly IModifiedCornerDofs modifiedCornerDofs;
		private readonly ConcurrentDictionary<int, FetiDPSubdomainDofs> subdomainDofsFetiDP;
		private readonly ConcurrentDictionary<int, PFetiDPSubdomainDofs> subdomainDofsPFetiDP;
		private readonly ConcurrentDictionary<int, IFetiDPSubdomainMatrixManager> subdomainMatricesFetiDP;

		public PFetiDPSolver(IComputeEnvironment environment, IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel,
			IPsmSubdomainMatrixManagerFactory<TMatrix> matrixFactoryPsm, bool explicitSubdomainMatrices,
			IPsmPreconditioner preconditioner, IPsmInterfaceProblemSolverFactory interfaceProblemSolverFactory, bool isHomogeneous,
			DdmLogger logger, ICornerDofSelection cornerDofs, IFetiDPCoarseProblemFactory coarseProblemFactory, 
			IFetiDPSubdomainMatrixManagerFactory<TMatrix> matrixFactoryFetiDP, PFetiDPReanalysisOptions reanalysis)
			: base(environment, model, algebraicModel, matrixFactoryPsm, explicitSubdomainMatrices, preconditioner,
				  interfaceProblemSolverFactory, isHomogeneous, logger, reanalysis, "PFETI-DP solver")
		{
			this.cornerDofs = cornerDofs;
			subdomainDofsFetiDP = new ConcurrentDictionary<int, FetiDPSubdomainDofs>();
			subdomainDofsPFetiDP = new ConcurrentDictionary<int, PFetiDPSubdomainDofs>();
			subdomainMatricesFetiDP = new ConcurrentDictionary<int, IFetiDPSubdomainMatrixManager>();
			environment.DoPerNode(subdomainID =>
			{
				SubdomainLinearSystem<TMatrix> linearSystem = algebraicModel.SubdomainLinearSystems[subdomainID];
				var dofsFetiDP = new FetiDPSubdomainDofs(linearSystem);
				var dofsPFetiDP = new PFetiDPSubdomainDofs(subdomainDofsPsm[subdomainID], dofsFetiDP);
				IFetiDPSubdomainMatrixManager matricesFetiDP = matrixFactoryFetiDP.CreateMatrixManager(linearSystem, dofsFetiDP);

				subdomainDofsFetiDP[subdomainID] = dofsFetiDP;
				subdomainDofsPFetiDP[subdomainID] = dofsPFetiDP;
				subdomainMatricesFetiDP[subdomainID] = matricesFetiDP;
			});

			this.coarseProblemFetiDP = coarseProblemFactory.CreateCoarseProblem(environment, algebraicModel.SubdomainTopology, 
				s => subdomainDofsFetiDP[s], s => subdomainMatricesFetiDP[s]);
			this.preconditioner = new PFetiDPPreconditioner(environment, () => base.boundaryDofIndexer, scaling,
				s => subdomainMatricesFetiDP[s], coarseProblemFetiDP, s => subdomainDofsPFetiDP[s]);

			if (reanalysis.GlobalCoarseProblemDofs)
			{
				modifiedCornerDofs = new GeneralModifiedCornerDofs(environment, s => subdomainDofsFetiDP[s]);
			}
			else
			{
				modifiedCornerDofs = new NullModifiedCornerDofs();
			}
		}

		protected override void CalcPreconditioner()
		{
			bool isFirstAnalysis = analysisIteration == 0;

			// Prepare subdomain-level dofs and matrices
			environment.DoPerNode(subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.SubdomainDofSubsets 
					|| reanalysis.ModifiedSubdomains.IsConnectivityModified(subdomainID))
				{
					#region debug
					//Console.WriteLine($"Processing corner & remainder dofs of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing corner & remainder dofs of subdomain {subdomainID}");
					#endregion
					subdomainDofsFetiDP[subdomainID].SeparateFreeDofsIntoCornerAndRemainder(cornerDofs);
					subdomainMatricesFetiDP[subdomainID].ReorderRemainderDofs();
					subdomainDofsPFetiDP[subdomainID].MapPsmFetiDPDofs();
				}
				else
				{
					Debug.Assert(!subdomainDofsFetiDP[subdomainID].IsEmpty);
					Debug.Assert(!subdomainDofsPFetiDP[subdomainID].IsEmpty);

				}

				if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices 
					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
				{
					#region debug
					//Console.WriteLine($"Processing corner & remainder submatrices of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing corner & remainder submatrices of subdomain {subdomainID}");
					#endregion
					subdomainMatricesFetiDP[subdomainID].HandleDofsWereModified();
					subdomainMatricesFetiDP[subdomainID].ExtractKrrKccKrc();
					subdomainMatricesFetiDP[subdomainID].InvertKrr();
				}
				else
				{
					Debug.Assert(!subdomainMatricesFetiDP[subdomainID].IsEmpty);
				}
			});

			// Setup optimizations if coarse dofs are the same as in previous analysis
			modifiedCornerDofs.Update(reanalysis.ModifiedSubdomains);

			// Prepare coarse problem
			coarseProblemFetiDP.FindCoarseProblemDofs(LoggerDdm, modifiedCornerDofs);
			coarseProblemFetiDP.PrepareMatricesForSolution();

			preconditioner.Calculate(environment, boundaryDofIndexer, interfaceProblemMatrix);
		}

		public new class Factory : PsmSolver<TMatrix>.Factory
		{
			private readonly ICornerDofSelection cornerDofs;

			public Factory(IComputeEnvironment environment, IPsmSubdomainMatrixManagerFactory<TMatrix> psmMatricesFactory,
				ICornerDofSelection cornerDofs, IFetiDPSubdomainMatrixManagerFactory<TMatrix> fetiDPMatricesFactory)
				: base(environment, psmMatricesFactory)
			{
				this.cornerDofs = cornerDofs;
				this.FetiDPMatricesFactory = fetiDPMatricesFactory;
				var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCSparse();
				this.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
				this.ReanalysisOptions = PFetiDPReanalysisOptions.CreateWithAllDisabled();
			}

			public IFetiDPCoarseProblemFactory CoarseProblemFactory { get; set; }

			public IFetiDPSubdomainMatrixManagerFactory<TMatrix> FetiDPMatricesFactory { get; set; }

			public new PFetiDPReanalysisOptions ReanalysisOptions { get; set; }

			public override PsmSolver<TMatrix> BuildSolver(IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel)
			{
				DdmLogger logger = EnableLogging ? new DdmLogger(environment, "PFETI-DP Solver", model.NumSubdomains) : null;
				return new PFetiDPSolver<TMatrix>(environment, model, algebraicModel, PsmMatricesFactory,
					ExplicitSubdomainMatrices, null, InterfaceProblemSolverFactory, IsHomogeneousProblem, logger,
					cornerDofs, CoarseProblemFactory, FetiDPMatricesFactory, ReanalysisOptions);
			}
		}
	}
}
