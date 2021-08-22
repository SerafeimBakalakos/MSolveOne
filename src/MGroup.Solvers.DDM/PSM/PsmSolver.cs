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
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.Output;
using MGroup.Solvers.DDM.PSM;
using MGroup.Solvers.DDM.PSM.Dofs;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.PSM.Preconditioning;
using MGroup.Solvers.DDM.PSM.Scaling;
using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
using MGroup.Solvers.DDM.PSM.Vectors;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.Solvers.Exceptions;
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
		protected readonly IPsmInterfaceProblemVectors interfaceProblemVectors;
		protected readonly IModel model;
		protected readonly string name;
		protected /*readonly*/ IPsmPreconditioner preconditioner; //TODO: Make this readonly as well.
		protected readonly PsmReanalysisOptions reanalysis;
		protected readonly IBoundaryDofScaling scaling;
		protected readonly ConcurrentDictionary<int, PsmSubdomainDofs> subdomainDofsPsm;
		protected readonly ConcurrentDictionary<int, IPsmSubdomainMatrixManager> subdomainMatricesPsm;
		protected readonly ISubdomainTopology subdomainTopology;
		protected readonly ConcurrentDictionary<int, PsmSubdomainVectors> subdomainVectors;

		protected int analysisIteration;
		protected DistributedOverlappingIndexer boundaryDofIndexer;

		protected PsmSolver(IComputeEnvironment environment, IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel, 
			IPsmSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory, 
			bool explicitSubdomainMatrices, IPsmPreconditioner preconditioner,
			IPsmInterfaceProblemSolverFactory interfaceProblemSolverFactory, bool isHomogeneous, DdmLogger logger,
			PsmReanalysisOptions reanalysis, string name = "PSM Solver")
		{
			this.name = name;
			this.environment = environment;
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.subdomainTopology = algebraicModel.SubdomainTopology;
			this.LinearSystem = algebraicModel.LinearSystem;
			this.preconditioner = preconditioner;
			this.reanalysis = reanalysis;

			this.subdomainDofsPsm = new ConcurrentDictionary<int, PsmSubdomainDofs>();
			this.subdomainMatricesPsm = new ConcurrentDictionary<int, IPsmSubdomainMatrixManager>();
			this.subdomainVectors = new ConcurrentDictionary<int, PsmSubdomainVectors>();
			environment.DoPerNode(subdomainID =>
			{
				SubdomainLinearSystem<TMatrix> linearSystem = algebraicModel.SubdomainLinearSystems[subdomainID];
				var dofs = new PsmSubdomainDofs(model.GetSubdomain(subdomainID), linearSystem, false);
				IPsmSubdomainMatrixManager matrices = matrixManagerFactory.CreateMatrixManager(linearSystem, dofs);
				var vectors = new PsmSubdomainVectors(linearSystem, dofs, matrices);

				subdomainDofsPsm[subdomainID] = dofs;
				subdomainMatricesPsm[subdomainID] = matrices;
				subdomainVectors[subdomainID] = vectors;
			});

			if (isHomogeneous)
			{
				this.scaling = new HomogeneousScaling(environment, s => subdomainDofsPsm[s], reanalysis);
			}
			else
			{
				this.scaling = new HeterogeneousScaling(environment, subdomainTopology,
					s => algebraicModel.SubdomainLinearSystems[s], s => subdomainDofsPsm[s]);
			}
			
			if (explicitSubdomainMatrices)
			{
				this.interfaceProblemMatrix = new PsmInterfaceProblemMatrixExplicit(
					environment, s => subdomainMatricesPsm[s], reanalysis);
			}
			else
			{
				this.interfaceProblemMatrix = new PsmInterfaceProblemMatrixImplicit(environment, 
					s => subdomainDofsPsm[s], s => subdomainMatricesPsm[s]);
			}

			if (reanalysis.RhsVectors)
			{
				this.interfaceProblemVectors = new PsmInterfaceProblemVectorsReanalysis(
					environment, subdomainVectors, reanalysis.ModifiedSubdomains);
			}
			else
			{
				this.interfaceProblemVectors = new PsmInterfaceProblemVectors(environment, subdomainVectors);
			}

			IPcgResidualConvergence convergenceCriterion;
			if (interfaceProblemSolverFactory.UseObjectiveConvergenceCriterion)
			{
				convergenceCriterion = new ObjectiveConvergenceCriterion<TMatrix>(
					environment, algebraicModel, s => subdomainVectors[s]);
			}
			else
			{
				convergenceCriterion = new RegularPcgConvergence();
			}
			this.interfaceProblemSolver = interfaceProblemSolverFactory.BuildIterativeMethod(convergenceCriterion);

			Logger = new SolverLogger(name);
			LoggerDdm = logger;

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

		public virtual void PreventFromOverwrittingSystemMatrices() {}

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
					#region debug
					//Console.WriteLine($"Processing boundary & internal dofs of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing boundary & internal dofs of subdomain {subdomainID}");
					#endregion
					subdomainDofsPsm[subdomainID].SeparateFreeDofsIntoBoundaryAndInternal();
					subdomainMatricesPsm[subdomainID].ReorderInternalDofs();
				}
				else
				{
					Debug.Assert(!subdomainDofsPsm[subdomainID].IsEmpty);
				}

				if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices 
					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
				{
					#region debug
					//Console.WriteLine($"Processing boundary & internal submatrices of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing boundary & internal submatrices of subdomain {subdomainID}");
					#endregion
					subdomainMatricesPsm[subdomainID].HandleDofsWereModified();
					subdomainMatricesPsm[subdomainID].ExtractKiiKbbKib();
					subdomainMatricesPsm[subdomainID].InvertKii();
				}
				else
				{
					Debug.Assert(!subdomainMatricesPsm[subdomainID].IsEmpty);
				}
			});

			// Intersubdomain dofs
			if (isFirstAnalysis || !reanalysis.InterfaceProblemIndexer)
			{
				this.boundaryDofIndexer = subdomainTopology.CreateDistributedVectorIndexer(
					s => subdomainDofsPsm[s].DofOrderingBoundary);
			}
			else
			{
				this.boundaryDofIndexer = subdomainTopology.RecreateDistributedVectorIndexer(
					s => subdomainDofsPsm[s].DofOrderingBoundary, this.boundaryDofIndexer,
					s => reanalysis.ModifiedSubdomains.IsConnectivityModified(s));
			}

			// Calculating scaling coefficients
			scaling.CalcScalingMatrices(boundaryDofIndexer);

			// Prepare subdomain-level vectors
			environment.DoPerNode(subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.RhsVectors 
					|| reanalysis.ModifiedSubdomains.IsRhsModified(subdomainID))
				{
					#region debug
					//Console.WriteLine($"Processing boundary & internal subvectors of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing boundary & internal subvectors of subdomain {subdomainID}");
					#endregion
					subdomainVectors[subdomainID].ExtractBoundaryInternalRhsVectors(
						fb => scaling.ScaleBoundaryRhsVector(subdomainID, fb));
				}
				else
				{
					Debug.Assert(!subdomainVectors[subdomainID].IsEmpty);
				}
			});

			// Prepare and solve the interface problem
			interfaceProblemMatrix.Calculate(boundaryDofIndexer);
			interfaceProblemVectors.CalcInterfaceRhsVector(boundaryDofIndexer);
			CalcPreconditioner();
			SolveInterfaceProblem();

			// Find the solution at all free dofs
			environment.DoPerNode(subdomainID =>
			{
				Vector subdomainBoundarySolution = interfaceProblemVectors.InterfaceProblemSolution.LocalVectors[subdomainID];
				subdomainVectors[subdomainID].CalcStoreSubdomainFreeSolution(subdomainBoundarySolution);
			});

			++analysisIteration;
			Logger.IncrementAnalysisStep();
		}

		protected virtual void CalcPreconditioner()
		{
			preconditioner.Calculate(environment, boundaryDofIndexer, interfaceProblemMatrix);
		}

		protected bool GuessInitialSolution()
		{
			// Initial guess of solution vector
			bool initalGuessIsZero = (analysisIteration == 0) || (!reanalysis.PreviousSolution);

			if (initalGuessIsZero)
			{
				#region log
				//Console.WriteLine("Allocating new solution vector.");
				//Debug.WriteLine("Allocating new solution vector.");
				#endregion

				interfaceProblemVectors.InterfaceProblemSolution = new DistributedOverlappingVector(boundaryDofIndexer);
			}
			else
			{
				DistributedOverlappingVector previousSolution = interfaceProblemVectors.InterfaceProblemSolution;
				if (boundaryDofIndexer.IsCompatibleVector(previousSolution))
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
					var newSolution = new DistributedOverlappingVector(boundaryDofIndexer, subdomainID =>
					{
						if (reanalysis.ModifiedSubdomains.IsConnectivityModified(subdomainID))
						{
							#region log
							//Console.WriteLine($"Reusing the previous solution subvector for subdomain {subdomainID}.");
							//Debug.WriteLine($"Reusing the previous solution subvector for subdomain {subdomainID}.");
							#endregion
							return Vector.CreateZero(boundaryDofIndexer.GetLocalComponent(subdomainID).NumEntries);
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

		protected void SolveInterfaceProblem()
		{
			bool initalGuessIsZero = GuessInitialSolution();

			// Solver the interface problem
			IterativeStatistics stats = interfaceProblemSolver.Solve(
				interfaceProblemMatrix.Matrix, preconditioner.Preconditioner, interfaceProblemVectors.InterfaceProblemRhs,
				interfaceProblemVectors.InterfaceProblemSolution, initalGuessIsZero);
			InterfaceProblemSolutionStats = stats;

			if (LoggerDdm != null)
			{
				LoggerDdm.LogProblemSize(0, algebraicModel.FreeDofIndexer.CountUniqueEntries());
				LoggerDdm.LogProblemSize(1, boundaryDofIndexer.CountUniqueEntries());
				LoggerDdm.LogSolverConvergenceData(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
			}
			Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
			Debug.WriteLine("Iterations for boundary problem = " + stats.NumIterationsRequired);
		}

		public class Factory
		{
			protected readonly IComputeEnvironment environment;

			public Factory(IComputeEnvironment environment, IPsmSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory)
			{
				this.environment = environment;
				DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
				EnableLogging = false;
				ExplicitSubdomainMatrices = false;
				InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg();
				IsHomogeneousProblem = true;
				PsmMatricesFactory = matrixManagerFactory; //new PsmSubdomainMatrixManagerSymmetricCSparse.Factory();
				Preconditioner = new PsmPreconditionerIdentity();
				ReanalysisOptions = PsmReanalysisOptions.CreateWithAllDisabled();
				SubdomainTopology = new SubdomainTopologyGeneral();
			}

			public IDofOrderer DofOrderer { get; set; }

			public bool EnableLogging { get; set; }

			public bool ExplicitSubdomainMatrices { get; set; }

			public IPsmInterfaceProblemSolverFactory InterfaceProblemSolverFactory { get; set; }

			public bool IsHomogeneousProblem { get; set; }

			public IPsmSubdomainMatrixManagerFactory<TMatrix> PsmMatricesFactory { get; }

			public IPsmPreconditioner Preconditioner { get; set; }

			public PsmReanalysisOptions ReanalysisOptions { get; set; }

			public ISubdomainTopology SubdomainTopology { get; set; }

			public DistributedAlgebraicModel<TMatrix> BuildAlgebraicModel(IModel model)
			{
				return new DistributedAlgebraicModel<TMatrix>(
					environment, model, DofOrderer, SubdomainTopology, PsmMatricesFactory.CreateAssembler(),
					ReanalysisOptions);
			}

			public virtual PsmSolver<TMatrix> BuildSolver(IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel)
			{
				DdmLogger logger = EnableLogging ? new DdmLogger(environment, "PSM Solver", model.NumSubdomains) : null;
				return new PsmSolver<TMatrix>(environment, model, algebraicModel, PsmMatricesFactory,
					ExplicitSubdomainMatrices, Preconditioner, InterfaceProblemSolverFactory, IsHomogeneousProblem,
					logger, ReanalysisOptions);
			}
		}
	}
}
