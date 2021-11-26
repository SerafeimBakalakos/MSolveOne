//using System;
//using System.Collections.Concurrent;
//using System.Collections.Generic;
//using System.Diagnostics;

//using MGroup.Environments;
//using MGroup.LinearAlgebra.Distributed.IterativeMethods;
//using MGroup.LinearAlgebra.Distributed.IterativeMethods.PCG;
//using MGroup.LinearAlgebra.Distributed.Overlapping;
//using MGroup.LinearAlgebra.Iterative;
//using MGroup.LinearAlgebra.Matrices;
//using MGroup.LinearAlgebra.Vectors;
//using MGroup.MSolve.Discretization;
//using MGroup.MSolve.Solution;
//using MGroup.MSolve.Solution.LinearSystem;
//using MGroup.Solvers.DDM.FetiDP.Dofs;
//using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
//using MGroup.Solvers.DDM.LagrangeMultipliers;
//using MGroup.Solvers.DDM.LinearSystem;
//using MGroup.Solvers.DDM.Output;
//using MGroup.Solvers.DofOrdering;
//using MGroup.Solvers.DofOrdering.Reordering;
//using MGroup.Solvers.Exceptions;
//using MGroup.Solvers.Logging;

//namespace MGroup.Solvers.DDM.FetiDP
//{
//	public class FetiDPSolver<TMatrix> : ISolver
//		where TMatrix : class, IMatrix
//	{
//		private readonly DistributedAlgebraicModel<TMatrix> algebraicModel;
//		private readonly ICornerDofSelection cornerDofs;
//		private readonly ICrossPointStrategy crossPointStrategy;
//		private readonly IComputeEnvironment environment;
//		private readonly IPsmInterfaceProblemMatrix interfaceProblemMatrix;
//		private readonly IDistributedIterativeMethod interfaceProblemSolver;
//		private readonly IPsmInterfaceProblemVectors interfaceProblemVectors;
//		private readonly IModel model;
//		private readonly string name;
//		//private /*readonly*/ IPsmPreconditioner preconditioner; //TODO: Make this readonly as well.
//		private readonly PsmReanalysisOptions reanalysis;
//		private readonly IBoundaryDofScaling scaling;
//		private readonly ConcurrentDictionary<int, FetiDPSubdomainDofs> subdomainDofsFetiDP;
//		private readonly ConcurrentDictionary<int, SubdomainLagranges> subdomainLagranges;
//		private readonly ConcurrentDictionary<int, IFetiDPSubdomainMatrixManager> subdomainMatricesFetiDP;
//		private readonly ISubdomainTopology subdomainTopology;
//		private readonly ConcurrentDictionary<int, PsmSubdomainVectors> subdomainVectors;
//		private readonly bool directSolverIsNative = false;

//		private int analysisIteration;
//		private DistributedOverlappingIndexer lagrangeVectorIndexer;

//		private FetiDPSolver(IComputeEnvironment environment, IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel, 
//			IFetiDPSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory, 
//			bool explicitSubdomainMatrices, IPsmPreconditioner preconditioner,
//			IPsmInterfaceProblemSolverFactory interfaceProblemSolverFactory,
//			ICornerDofSelection cornerDofs, ICrossPointStrategy crossPointStrategy,
//			bool isHomogeneous, DdmLogger logger,
//			PsmReanalysisOptions reanalysis, string name = "FETI-DP Solver")
//		{
//			this.name = name;
//			this.environment = environment;
//			this.model = model;
//			this.algebraicModel = algebraicModel;
//			this.cornerDofs = cornerDofs;
//			this.crossPointStrategy = crossPointStrategy;
//			this.subdomainTopology = algebraicModel.SubdomainTopology;
//			this.LinearSystem = algebraicModel.LinearSystem;
//			this.preconditioner = preconditioner;
//			this.reanalysis = reanalysis;

//			this.subdomainDofsFetiDP = new ConcurrentDictionary<int, FetiDPSubdomainDofs>();
//			this.subdomainMatricesFetiDP = new ConcurrentDictionary<int, IFetiDPSubdomainMatrixManager>();
//			this.subdomainVectors = new ConcurrentDictionary<int, PsmSubdomainVectors>();
//			environment.DoPerNode(subdomainID =>
//			{
//				SubdomainLinearSystem<TMatrix> linearSystem = algebraicModel.SubdomainLinearSystems[subdomainID];
//				var dofs = new FetiDPSubdomainDofs(model.GetSubdomain(subdomainID), linearSystem);
//				var lagranges = new SubdomainLagranges(model, subdomainID, subdomainTopology, dofs, crossPointStrategy);
//				IFetiDPSubdomainMatrixManager matrices = matrixManagerFactory.CreateMatrixManager(linearSystem, dofs);
//				var vectors = new PsmSubdomainVectors(linearSystem, dofs, matrices);

//				subdomainDofsFetiDP[subdomainID] = dofs;
//				subdomainLagranges[subdomainID] = lagranges;
//				subdomainMatricesFetiDP[subdomainID] = matrices;
//				subdomainVectors[subdomainID] = vectors;
//			});

//			if (isHomogeneous)
//			{
//				this.scaling = new HomogeneousScaling(environment, s => subdomainDofsFetiDP[s], reanalysis);
//			}
//			else
//			{
//				this.scaling = new HeterogeneousScaling(environment, subdomainTopology,
//					s => algebraicModel.SubdomainLinearSystems[s], s => subdomainDofsFetiDP[s]);
//			}
			
//			if (explicitSubdomainMatrices)
//			{
//				this.interfaceProblemMatrix = new PsmInterfaceProblemMatrixExplicit(
//					environment, s => subdomainMatricesFetiDP[s], reanalysis);
//			}
//			else
//			{
//				this.interfaceProblemMatrix = new PsmInterfaceProblemMatrixImplicit(environment, 
//					s => subdomainDofsFetiDP[s], s => subdomainMatricesFetiDP[s]);
//			}

//			if (reanalysis.RhsVectors)
//			{
//				this.interfaceProblemVectors = new PsmInterfaceProblemVectorsReanalysis(
//					environment, subdomainVectors, reanalysis.ModifiedSubdomains);
//			}
//			else
//			{
//				this.interfaceProblemVectors = new PsmInterfaceProblemVectors(environment, subdomainVectors);
//			}

//			IPcgResidualConvergence convergenceCriterion;
//			if (interfaceProblemSolverFactory.UseObjectiveConvergenceCriterion)
//			{
//				convergenceCriterion = new ObjectiveConvergenceCriterion<TMatrix>(
//					environment, algebraicModel, s => subdomainVectors[s]);
//			}
//			else
//			{
//				convergenceCriterion = new RegularPcgConvergence();
//			}
//			this.interfaceProblemSolver = interfaceProblemSolverFactory.BuildIterativeMethod(convergenceCriterion);

//			Logger = new SolverLogger(name);
//			LoggerDdm = logger;

//			if (matrixManagerFactory is FetiDPSubdomainMatrixManagerSymmetricSuiteSparse.Factory)
//			{
//				directSolverIsNative = true;
//			}
//			else
//			{
//				directSolverIsNative = false;
//			}

//			analysisIteration = 0;
//		}

//		public IterativeStatistics InterfaceProblemSolutionStats { get; private set; }

//		public IGlobalLinearSystem LinearSystem { get; }

//		public ISolverLogger Logger { get; }

//		public DdmLogger LoggerDdm { get; }

//		public string Name => name;


//		public virtual void HandleMatrixWillBeSet()
//		{
//		}

//		public virtual void PreventFromOverwrittingSystemMatrices() {}

//		public virtual void Solve() 
//		{
//			bool isFirstAnalysis = analysisIteration == 0;

//			if (LoggerDdm != null)
//			{
//				LoggerDdm.IncrementAnalysisIteration();
//			}

//			// Prepare subdomain-level dofs and matrices
//			environment.DoPerNode(subdomainID =>
//			{
//				if (isFirstAnalysis || !reanalysis.SubdomainDofSubsets 
//					|| reanalysis.ModifiedSubdomains.IsConnectivityModified(subdomainID))
//				{
//					#region log
//					//Console.WriteLine($"Processing corner, boundary-remainder & internal dofs of subdomain {subdomainID}");
//					//Debug.WriteLine($"Processing corner, boundary-remainder & internal dofs of subdomain {subdomainID}");
//					#endregion
//					subdomainDofsFetiDP[subdomainID].SeparateAllFreeDofs(cornerDofs);
//					subdomainMatricesFetiDP[subdomainID].ReorderRemainderDofs();
//					subdomainLagranges[subdomainID].DefineSubdomainLagrangeMultipliers();
//					subdomainLagranges[subdomainID].CalcMatrixDr();
//				}
//				else
//				{
//					Debug.Assert(!subdomainDofsFetiDP[subdomainID].IsEmpty);
//				}

//				if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices 
//					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
//				{
//					#region log
//					//Console.WriteLine($"Processing corner, boundary-remainder & internal submatrices of subdomain {subdomainID}");
//					//Debug.WriteLine($"Processing corner, boundary-remainder & internal submatrices of subdomain {subdomainID}");
//					#endregion
//					subdomainMatricesFetiDP[subdomainID].HandleDofsWereModified();
//					subdomainMatricesFetiDP[subdomainID].ExtractKrrKccKrc();
//					//subdomainMatricesPsm[subdomainID].InvertKii();
//				}
//				else
//				{
//					Debug.Assert(!subdomainMatricesFetiDP[subdomainID].IsEmpty);
//				}
//			});

//			//TODO: This should be done together with the extraction. However SuiteSparse already uses multiple threads and should
//			//		not be parallelized at subdomain level too. Instead environment.DoPerNode should be able to run tasks serially by reading a flag.
//			if (directSolverIsNative)
//			{
//				environment.DoPerNodeSerially(subdomainID =>
//				{
//					if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices
//					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
//					{
//						subdomainMatricesFetiDP[subdomainID].InvertKrr();
//					}
//				});
//			}
//			else
//			{
//				environment.DoPerNode(subdomainID =>
//				{
//					if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices
//					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
//					{
//						subdomainMatricesFetiDP[subdomainID].InvertKrr();
//					}
//				});
//			}

//			// Intersubdomain lagrange multipliers
//			if (true/*isFirstAnalysis || !reanalysis.InterfaceProblemIndexer*/)
//			{
//				this.lagrangeVectorIndexer = new DistributedOverlappingIndexer(environment);
//				environment.DoPerNode(subdomainID =>
//				{
//					subdomainLagranges[subdomainID].FindCommonLagrangesWithNeighbors();
//					subdomainLagranges[subdomainID].InitializeDistributedVectorIndexer(
//						this.lagrangeVectorIndexer.GetLocalComponent(subdomainID));
//				});
//			}
//			else
//			{
//			}

//			// Calculating scaling coefficients
//			scaling.CalcScalingMatrices(lagrangeVectorIndexer);

//			// Prepare subdomain-level vectors
//			environment.DoPerNode(subdomainID =>
//			{
//				if (isFirstAnalysis || !reanalysis.RhsVectors 
//					|| reanalysis.ModifiedSubdomains.IsRhsModified(subdomainID))
//				{
//					#region log
//					//Console.WriteLine($"Processing corner, boundary-remainder & internal subvectors of subdomain {subdomainID}");
//					//Debug.WriteLine($"Processing corner, boundary-remainder & internal subvectors of subdomain {subdomainID}");
//					#endregion
//					subdomainVectors[subdomainID].ExtractBoundaryInternalRhsVectors(
//						fb => scaling.ScaleBoundaryRhsVector(subdomainID, fb));
//				}
//				else
//				{
//					Debug.Assert(!subdomainVectors[subdomainID].IsEmpty);
//				}
//			});

//			// Prepare and solve the interface problem
//			interfaceProblemMatrix.Calculate(lagrangeVectorIndexer);
//			interfaceProblemVectors.CalcInterfaceRhsVector(lagrangeVectorIndexer);
//			CalcPreconditioner();
//			SolveInterfaceProblem();

//			// Find the solution at all free dofs
//			environment.DoPerNode(subdomainID =>
//			{
//				Vector subdomainBoundarySolution = interfaceProblemVectors.InterfaceProblemSolution.LocalVectors[subdomainID];
//				subdomainVectors[subdomainID].CalcStoreSubdomainFreeSolution(subdomainBoundarySolution);
//			});

//			++analysisIteration;
//			Logger.IncrementAnalysisStep();
//		}

//		private void CalcPreconditioner()
//		{
//			preconditioner.Calculate(environment, lagrangeVectorIndexer, interfaceProblemMatrix);
//		}

//		private bool GuessInitialSolution()
//		{
//			// Initial guess of solution vector
//			bool initalGuessIsZero = (analysisIteration == 0) || (!reanalysis.PreviousSolution);

//			if (initalGuessIsZero)
//			{
//				#region log
//				//Console.WriteLine("Allocating new solution vector.");
//				//Debug.WriteLine("Allocating new solution vector.");
//				#endregion

//				interfaceProblemVectors.InterfaceProblemSolution = new DistributedOverlappingVector(lagrangeVectorIndexer);
//			}
//			else
//			{
//				DistributedOverlappingVector previousSolution = interfaceProblemVectors.InterfaceProblemSolution;
//				if (lagrangeVectorIndexer.IsCompatibleVector(previousSolution))
//				{
//					// Do nothing to modify the stored solution vector.
//					#region log
//					//Console.WriteLine("Reusing the previous solution vector.");
//					//Debug.WriteLine("Reusing the previous solution vector.");
//					#endregion
//				}
//				else
//				{
//					// The dof orderings of some subdomains may remain the same, in which case we can reuse the previous values.
//					var newSolution = new DistributedOverlappingVector(lagrangeVectorIndexer, subdomainID =>
//					{
//						if (reanalysis.ModifiedSubdomains.IsConnectivityModified(subdomainID))
//						{
//							#region log
//							//Console.WriteLine($"Reusing the previous solution subvector for subdomain {subdomainID}.");
//							//Debug.WriteLine($"Reusing the previous solution subvector for subdomain {subdomainID}.");
//							#endregion
//							return Vector.CreateZero(lagrangeVectorIndexer.GetLocalComponent(subdomainID).NumEntries);
//						}
//						else
//						{
//							return previousSolution.LocalVectors[subdomainID];
//						}
//					});
					
//					interfaceProblemVectors.InterfaceProblemSolution = newSolution;
//				}
//			}

//			return initalGuessIsZero;
//		}

//		private void SolveInterfaceProblem()
//		{
//			bool initalGuessIsZero = GuessInitialSolution();

//			// Solver the interface problem
//			IterativeStatistics stats = interfaceProblemSolver.Solve(
//				interfaceProblemMatrix.Matrix, preconditioner.Preconditioner, interfaceProblemVectors.InterfaceProblemRhs,
//				interfaceProblemVectors.InterfaceProblemSolution, initalGuessIsZero);
//			InterfaceProblemSolutionStats = stats;

//			if (LoggerDdm != null)
//			{
//				LoggerDdm.LogProblemSize(0, algebraicModel.FreeDofIndexer.CountUniqueEntries());
//				LoggerDdm.LogProblemSize(1, lagrangeVectorIndexer.CountUniqueEntries());
//				LoggerDdm.LogSolverConvergenceData(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
//			}
//			Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
//			Debug.WriteLine("Iterations for boundary problem = " + stats.NumIterationsRequired);
//		}

//		public class Factory
//		{
//			private readonly IComputeEnvironment environment;

//			public Factory(IComputeEnvironment environment, IPsmSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory)
//			{
//				this.environment = environment;
//				DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
//				EnableLogging = false;
//				ExplicitSubdomainMatrices = false;
//				InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg();
//				IsHomogeneousProblem = true;
//				PsmMatricesFactory = matrixManagerFactory; //new PsmSubdomainMatrixManagerSymmetricCSparse.Factory();
//				Preconditioner = new PsmPreconditionerIdentity();
//				ReanalysisOptions = PsmReanalysisOptions.CreateWithAllDisabled();
//				SubdomainTopology = new SubdomainTopologyGeneral();
//			}

//			public IDofOrderer DofOrderer { get; set; }

//			public bool EnableLogging { get; set; }

//			public bool ExplicitSubdomainMatrices { get; set; }

//			public IPsmInterfaceProblemSolverFactory InterfaceProblemSolverFactory { get; set; }

//			public bool IsHomogeneousProblem { get; set; }

//			public IPsmSubdomainMatrixManagerFactory<TMatrix> PsmMatricesFactory { get; }

//			public IPsmPreconditioner Preconditioner { get; set; }

//			public PsmReanalysisOptions ReanalysisOptions { get; set; }

//			public ISubdomainTopology SubdomainTopology { get; set; }

//			public DistributedAlgebraicModel<TMatrix> BuildAlgebraicModel(IModel model)
//			{
//				return new DistributedAlgebraicModel<TMatrix>(
//					environment, model, DofOrderer, SubdomainTopology, PsmMatricesFactory.CreateAssembler(),
//					ReanalysisOptions);
//			}

//			public virtual PsmSolver<TMatrix> BuildSolver(IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel)
//			{
//				DdmLogger logger = EnableLogging ? new DdmLogger(environment, "PSM Solver", model.NumSubdomains) : null;
//				return new PsmSolver<TMatrix>(environment, model, algebraicModel, PsmMatricesFactory,
//					ExplicitSubdomainMatrices, Preconditioner, InterfaceProblemSolverFactory, IsHomogeneousProblem,
//					logger, ReanalysisOptions);
//			}
//		}
//	}
//}
