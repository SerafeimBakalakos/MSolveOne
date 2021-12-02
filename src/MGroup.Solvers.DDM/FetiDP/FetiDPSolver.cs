using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;

using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.IterativeMethods;
using MGroup.LinearAlgebra.Distributed.IterativeMethods.PCG;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Matrices.Operators;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.InterfaceProblem;
using MGroup.Solvers.DDM.FetiDP.Preconditioning;
using MGroup.Solvers.DDM.FetiDP.Scaling;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
using MGroup.Solvers.DDM.FetiDP.Vectors;
using MGroup.Solvers.DDM.LagrangeMultipliers;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.Output;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.Solvers.Exceptions;
using MGroup.Solvers.Logging;

namespace MGroup.Solvers.DDM.FetiDP
{
	public class FetiDPSolver<TMatrix> : ISolver
		where TMatrix : class, IMatrix
	{
		private readonly DistributedAlgebraicModel<TMatrix> algebraicModel;
		private readonly IFetiDPCoarseProblem coarseProblem;
		private readonly ICornerDofSelection cornerDofs;
		private readonly ICrossPointStrategy crossPointStrategy;
		private readonly IComputeEnvironment environment;
		private readonly IFetiDPInterfaceProblemMatrix interfaceProblemMatrix;
		private readonly IDistributedIterativeMethod interfaceProblemSolver;
		private readonly IFetiDPInterfaceProblemVectors interfaceProblemVectors;
		private readonly IModel model;
		private readonly IModifiedCornerDofs modifiedCornerDofs;
		private readonly string name;
		private readonly IFetiDPPreconditioner preconditioner;
		private readonly FetiDPReanalysisOptions reanalysis;
		private readonly IFetiDPScaling scaling;
		private readonly FetiDPSolutionRecovery solutionRecovery;
		private readonly ConcurrentDictionary<int, FetiDPSubdomainDofs> subdomainDofs;
		private readonly ConcurrentDictionary<int, SubdomainLagranges> subdomainLagranges;
		private readonly ConcurrentDictionary<int, IFetiDPSubdomainMatrixManager> subdomainMatrices;
		private readonly ISubdomainTopology subdomainTopology;
		private readonly ConcurrentDictionary<int, FetiDPSubdomainRhsVectors> subdomainVectors;
		private readonly bool directSolverIsNative = false;

		private int analysisIteration;
		private DistributedOverlappingIndexer lagrangeVectorIndexer;

		private FetiDPSolver(IComputeEnvironment environment, IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel,
			IFetiDPSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory,
			bool explicitSubdomainMatrices, IFetiDPPreconditioner preconditioner,
			IFetiDPInterfaceProblemSolverFactory interfaceProblemSolverFactory, ICornerDofSelection cornerDofs,
			IFetiDPCoarseProblemFactory coarseProblemFactory, ICrossPointStrategy crossPointStrategy,
			bool isHomogeneous, DdmLogger logger,
			FetiDPReanalysisOptions reanalysis, string name = "FETI-DP Solver")
		{
			this.name = name;
			this.environment = environment;
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.cornerDofs = cornerDofs;
			this.crossPointStrategy = crossPointStrategy;
			this.subdomainTopology = algebraicModel.SubdomainTopology;
			this.LinearSystem = algebraicModel.LinearSystem;
			this.preconditioner = preconditioner;
			this.reanalysis = reanalysis;

			this.subdomainDofs = new ConcurrentDictionary<int, FetiDPSubdomainDofs>();
			this.subdomainLagranges = new ConcurrentDictionary<int, SubdomainLagranges>();
			this.subdomainMatrices = new ConcurrentDictionary<int, IFetiDPSubdomainMatrixManager>();
			this.subdomainVectors = new ConcurrentDictionary<int, FetiDPSubdomainRhsVectors>();
			environment.DoPerNode(subdomainID =>
			{
				SubdomainLinearSystem<TMatrix> linearSystem = algebraicModel.SubdomainLinearSystems[subdomainID];
				var dofs = new FetiDPSubdomainDofs(model.GetSubdomain(subdomainID), linearSystem);
				var lagranges = new SubdomainLagranges(model, subdomainID, subdomainTopology, dofs, crossPointStrategy);
				IFetiDPSubdomainMatrixManager matrices = matrixManagerFactory.CreateMatrixManager(linearSystem, dofs);
				var vectors = new FetiDPSubdomainRhsVectors(linearSystem, dofs, lagranges, matrices);

				subdomainDofs[subdomainID] = dofs;
				subdomainLagranges[subdomainID] = lagranges;
				subdomainMatrices[subdomainID] = matrices;
				subdomainVectors[subdomainID] = vectors;
			});

			this.coarseProblem = coarseProblemFactory.CreateCoarseProblem(environment, algebraicModel.SubdomainTopology,
				s => subdomainDofs[s], s => subdomainMatrices[s]);
			if (reanalysis.GlobalCoarseProblemDofs)
			{
				modifiedCornerDofs = new GeneralModifiedCornerDofs(environment, s => subdomainDofs[s]);
			}
			else
			{
				modifiedCornerDofs = new NullModifiedCornerDofs();
			}

			if (isHomogeneous)
			{
				this.scaling = new HomogeneousScaling(
					environment, model, s => subdomainDofs[s], crossPointStrategy, reanalysis);
			}
			else
			{
				throw new NotImplementedException();
			}

			if (explicitSubdomainMatrices)
			{
				throw new NotImplementedException();
			}
			else
			{
				this.interfaceProblemMatrix = new FetiDPInterfaceProblemMatrixImplicit(
					environment, coarseProblem, subdomainLagranges, subdomainMatrices);
			}

			if (reanalysis.RhsVectors)
			{
				throw new NotImplementedException();
			}
			else
			{
				this.interfaceProblemVectors = new FetiDPInterfaceProblemVectors(
					environment, coarseProblem, subdomainLagranges, subdomainMatrices, subdomainVectors);
			}

			this.solutionRecovery = new FetiDPSolutionRecovery(environment, coarseProblem, scaling,
				s => subdomainDofs[s], s => subdomainLagranges[s], s => subdomainMatrices[s], s => subdomainVectors[s]);

			IPcgResidualConvergence convergenceCriterion;
			if (interfaceProblemSolverFactory.UseObjectiveConvergenceCriterion)
			{
				convergenceCriterion = new ObjectiveConvergenceCriterion<TMatrix>(algebraicModel, solutionRecovery);
			}
			else
			{
				convergenceCriterion = new ApproximatePcgResidualConvergence<TMatrix>(algebraicModel);
			}
			this.interfaceProblemSolver = interfaceProblemSolverFactory.BuildIterativeMethod(convergenceCriterion);

			Logger = new SolverLogger(name);
			LoggerDdm = logger;

			if (matrixManagerFactory is FetiDPSubdomainMatrixManagerSymmetricSuiteSparse.Factory)
			{
				directSolverIsNative = true;
			}
			else
			{
				directSolverIsNative = false;
			}

			analysisIteration = 0;
		}

		public IterativeStatistics InterfaceProblemSolutionStats { get; private set; }

		public IGlobalLinearSystem LinearSystem { get; }

		public ISolverLogger Logger { get; }

		public DdmLogger LoggerDdm { get; }

		public string Name => name;


		public virtual void HandleMatrixWillBeSet()
		{
		}

		public virtual void PreventFromOverwrittingSystemMatrices() { }

		public virtual void Solve()
		{
			bool isFirstAnalysis = analysisIteration == 0;

			if (LoggerDdm != null)
			{
				LoggerDdm.IncrementAnalysisIteration();
			}

			// Prepare subdomain-level dofs and matrices
			environment.DoPerNode(subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.SubdomainDofSubsets
					|| reanalysis.ModifiedSubdomains.IsConnectivityModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Processing corner, boundary-remainder & internal dofs of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing corner, boundary-remainder & internal dofs of subdomain {subdomainID}");
					#endregion
					subdomainDofs[subdomainID].SeparateAllFreeDofs(cornerDofs);
					subdomainMatrices[subdomainID].ReorderRemainderDofs();
					subdomainLagranges[subdomainID].DefineSubdomainLagrangeMultipliers();
					subdomainLagranges[subdomainID].CalcSignedBooleanMatrices();
				}
				else
				{
					Debug.Assert(!subdomainDofs[subdomainID].IsEmpty);
				}

				if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices
					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Processing corner, boundary-remainder & internal submatrices of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing corner, boundary-remainder & internal submatrices of subdomain {subdomainID}");
					#endregion
					subdomainMatrices[subdomainID].HandleDofsWereModified();
					subdomainMatrices[subdomainID].ExtractKrrKccKrc();
					//subdomainMatricesPsm[subdomainID].InvertKii();
				}
				else
				{
					Debug.Assert(!subdomainMatrices[subdomainID].IsEmpty);
				}
			});

			//TODO: This should be done together with the extraction. However SuiteSparse already uses multiple threads and should
			//		not be parallelized at subdomain level too. Instead environment.DoPerNode should be able to run tasks serially by reading a flag.
			if (directSolverIsNative)
			{
				environment.DoPerNodeSerially(subdomainID =>
				{
					if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices
					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
					{
						subdomainMatrices[subdomainID].InvertKrr();
					}
				});
			}
			else
			{
				environment.DoPerNode(subdomainID =>
				{
					if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices
					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
					{
						subdomainMatrices[subdomainID].InvertKrr();
					}
				});
			}

			// Intersubdomain lagrange multipliers
			if (true/*isFirstAnalysis || !reanalysis.InterfaceProblemIndexer*/)
			{
				this.lagrangeVectorIndexer = new DistributedOverlappingIndexer(environment);
				environment.DoPerNode(subdomainID =>
				{
					subdomainLagranges[subdomainID].FindCommonLagrangesWithNeighbors();
					subdomainLagranges[subdomainID].InitializeDistributedVectorIndexer(
						this.lagrangeVectorIndexer.GetLocalComponent(subdomainID));
				});
			}
			else
			{
			}

			// Calculating scaling coefficients
			scaling.CalcScalingMatrices();

			// Prepare subdomain-level vectors
			environment.DoPerNode(subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.RhsVectors
					|| reanalysis.ModifiedSubdomains.IsRhsModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Processing corner, boundary-remainder & internal subvectors of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing corner, boundary-remainder & internal subvectors of subdomain {subdomainID}");
					#endregion
					subdomainVectors[subdomainID].ExtractRhsSubvectors(
						fb => scaling.ScaleSubdomainRhsVector(subdomainID, fb));
					subdomainVectors[subdomainID].CalcCondensedRhsVector();
				}
				else
				{
					Debug.Assert(!subdomainVectors[subdomainID].IsEmpty);
				}
			});

			// Setup optimizations if coarse dofs are the same as in previous analysis
			modifiedCornerDofs.Update(reanalysis.ModifiedSubdomains);

			// Prepare coarse problem
			coarseProblem.FindCoarseProblemDofs(LoggerDdm, modifiedCornerDofs);
			coarseProblem.PrepareMatricesForSolution();

			// Prepare and solve the interface problem
			interfaceProblemMatrix.Calculate(lagrangeVectorIndexer);
			interfaceProblemVectors.CalcInterfaceRhsVector(lagrangeVectorIndexer);
			preconditioner.Initialize(
				environment, lagrangeVectorIndexer, s => subdomainLagranges[s], s => subdomainMatrices[s], scaling);
			SolveInterfaceProblem();

			// Having found the lagrange multipliers, now calculate the solution in term of primal dofs
			solutionRecovery.CalcPrimalSolution(
				interfaceProblemVectors.InterfaceProblemSolution, algebraicModel.LinearSystem.Solution);

			++analysisIteration;
			Logger.IncrementAnalysisStep();
		}

		private bool GuessInitialSolution()
		{
			// Initial guess of solution vector
			bool initalGuessIsZero = (analysisIteration == 0) || (!reanalysis.PreviousSolution);

			if (initalGuessIsZero)
			{
				#region log
				//Console.WriteLine("Allocating new solution vector.");
				//Debug.WriteLine("Allocating new solution vector.");
				#endregion

				interfaceProblemVectors.InterfaceProblemSolution = new DistributedOverlappingVector(lagrangeVectorIndexer);
			}
			else
			{
				DistributedOverlappingVector previousSolution = interfaceProblemVectors.InterfaceProblemSolution;
				if (lagrangeVectorIndexer.IsCompatibleVector(previousSolution))
				{
					// Do nothing to modify the stored solution vector.
					#region log
					//Console.WriteLine("Reusing the previous solution vector.");
					//Debug.WriteLine("Reusing the previous solution vector.");
					#endregion
				}
				else
				{
					// The dof orderings of some subdomains may remain the same, in which case we can reuse the previous values.
					var newSolution = new DistributedOverlappingVector(lagrangeVectorIndexer, subdomainID =>
					{
						if (reanalysis.ModifiedSubdomains.IsConnectivityModified(subdomainID))
						{
							#region log
							//Console.WriteLine($"Reusing the previous solution subvector for subdomain {subdomainID}.");
							//Debug.WriteLine($"Reusing the previous solution subvector for subdomain {subdomainID}.");
							#endregion
							return Vector.CreateZero(lagrangeVectorIndexer.GetLocalComponent(subdomainID).NumEntries);
						}
						else
						{
							return previousSolution.LocalVectors[subdomainID];
						}
					});

					interfaceProblemVectors.InterfaceProblemSolution = newSolution;
				}
			}

			return initalGuessIsZero;
		}

		private void SolveInterfaceProblem()
		{
			bool initalGuessIsZero = GuessInitialSolution();

			// Solver the interface problem
			IterativeStatistics stats = interfaceProblemSolver.Solve(
				interfaceProblemMatrix, preconditioner, interfaceProblemVectors.InterfaceProblemRhs,
				interfaceProblemVectors.InterfaceProblemSolution, initalGuessIsZero);
			InterfaceProblemSolutionStats = stats;

			if (LoggerDdm != null)
			{
				LoggerDdm.LogProblemSize(0, algebraicModel.FreeDofIndexer.CountUniqueEntries());
				LoggerDdm.LogProblemSize(1, lagrangeVectorIndexer.CountUniqueEntries());
				LoggerDdm.LogSolverConvergenceData(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
			}
			Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
			Debug.WriteLine("Iterations for boundary problem = " + stats.NumIterationsRequired);
		}

		public class Factory
		{
			private readonly ICornerDofSelection cornerDofs;
			private readonly IComputeEnvironment environment;

			public Factory(IComputeEnvironment environment, ICornerDofSelection cornerDofs,
				IFetiDPSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory)
			{
				this.environment = environment;
				this.cornerDofs = cornerDofs;

				CrossPointStrategy = new FullyRedundantLagranges();
				DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
				EnableLogging = false;
				ExplicitSubdomainMatrices = false;
				InterfaceProblemSolverFactory = new FetiDPInterfaceProblemSolverFactoryPcg();
				IsHomogeneousProblem = true;
				FetiDPMatricesFactory = matrixManagerFactory;
				Preconditioner = new FetiDPDirichletPreconditioner();
				var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCSparse();
				this.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
				ReanalysisOptions = FetiDPReanalysisOptions.CreateWithAllDisabled();
				SubdomainTopology = new SubdomainTopologyGeneral();
			}

			public IFetiDPCoarseProblemFactory CoarseProblemFactory { get; set; }

			public ICrossPointStrategy CrossPointStrategy { get; set; }

			public IDofOrderer DofOrderer { get; set; }

			public bool EnableLogging { get; set; }

			public bool ExplicitSubdomainMatrices { get; set; }

			public IFetiDPInterfaceProblemSolverFactory InterfaceProblemSolverFactory { get; set; }

			public bool IsHomogeneousProblem { get; set; }

			public IFetiDPSubdomainMatrixManagerFactory<TMatrix> FetiDPMatricesFactory { get; }

			public IFetiDPPreconditioner Preconditioner { get; set; }

			public FetiDPReanalysisOptions ReanalysisOptions { get; set; }

			public ISubdomainTopology SubdomainTopology { get; set; }

			public DistributedAlgebraicModel<TMatrix> BuildAlgebraicModel(IModel model)
			{
				return new DistributedAlgebraicModel<TMatrix>(
					environment, model, DofOrderer, SubdomainTopology, FetiDPMatricesFactory.CreateAssembler(),
					ReanalysisOptions);
			}

			public virtual FetiDPSolver<TMatrix> BuildSolver(IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel)
			{
				DdmLogger logger = EnableLogging ? new DdmLogger(environment, "PSM Solver", model.NumSubdomains) : null;
				return new FetiDPSolver<TMatrix>(environment, model, algebraicModel, FetiDPMatricesFactory,
					ExplicitSubdomainMatrices, Preconditioner, InterfaceProblemSolverFactory, cornerDofs, CoarseProblemFactory,
					CrossPointStrategy, IsHomogeneousProblem, logger, ReanalysisOptions);
			}
		}
	}
}
