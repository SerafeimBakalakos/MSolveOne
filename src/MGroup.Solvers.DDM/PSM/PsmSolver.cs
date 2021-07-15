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

namespace MGroup.Solvers.DDM.Psm
{
	public class PsmSolver : ISolver
	{
		protected readonly IDofOrderer dofOrderer;
		protected readonly PsmDofManager dofManagerPsm;
		protected readonly IComputeEnvironment environment;
		protected readonly IPsmInterfaceProblemMatrix interfaceProblemMatrix;
		protected readonly IDistributedIterativeMethod interfaceProblemSolver;
		protected readonly PsmInterfaceProblemVectors interfaceProblemVectors;
		protected readonly ConcurrentDictionary<int, ISubdomainMatrixManager> matrixManagersBasic;
		protected readonly ConcurrentDictionary<int, IPsmSubdomainMatrixManager> matrixManagersPsm;
		protected readonly IModel model;
		protected readonly string name;
		protected readonly IPsmPreconditioner preconditioner;

		protected readonly Dictionary<int, PsmSubdomainVectors> subdomainVectors;
		//protected readonly IPsmRhsVectorManager rhsVectorManager;
		//protected readonly IPsmSolutionVectorManager solutionVectorManager;
		protected readonly IBoundaryDofScaling stiffnessDistribution;
		protected readonly SubdomainTopology subdomainTopology;

		private DistributedOverlappingIndexer indexer; //TODOMPI: Perhaps this should be accessed from DofSeparator

		protected PsmSolver(IComputeEnvironment environment, IModel model, SubdomainTopology subdomainTopology,
			IDofOrderer dofOrderer, IPsmSubdomainMatrixManagerFactory matrixManagerFactory, 
			bool explicitSubdomainMatrices, IPsmPreconditioner preconditioner,
			IDistributedIterativeMethod interfaceProblemSolver, bool isHomogeneous, string name = "PSM Solver")
		{
			this.name = name;
			this.environment = environment;

			this.model = model;
			this.subdomainTopology = subdomainTopology;
			this.dofOrderer = dofOrderer;
			this.preconditioner = preconditioner;
			this.interfaceProblemSolver = interfaceProblemSolver;

			this.dofManagerPsm = new PsmDofManager(environment, model, subdomainTopology, false);

			matrixManagersBasic = new ConcurrentDictionary<int, ISubdomainMatrixManager>();
			matrixManagersPsm = new ConcurrentDictionary<int, IPsmSubdomainMatrixManager>();
			environment.DoPerNode(subdomainID =>
			{
				ISubdomain subdomain = model.GetSubdomain(subdomainID);
				PsmSubdomainDofs subdomainDofs = dofManagerPsm.GetSubdomainDofs(subdomainID);
				(ISubdomainMatrixManager matrixManagerBasic, IPsmSubdomainMatrixManager matrixManagerPsm) = 
					matrixManagerFactory.CreateMatrixManagers(subdomain, subdomainDofs);
				matrixManagersBasic[subdomainID] = matrixManagerBasic;
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
					dofManagerPsm.GetSubdomainDofs(subdomainID), LinearSystem/*s[subdomainID]*/, 
					matrixManagersBasic[subdomainID], matrixManagersPsm[subdomainID]));

			this.interfaceProblemVectors = new PsmInterfaceProblemVectors(environment, subdomainVectors);

			Logger = new SolverLogger(name);
		}

		//public IReadOnlyDictionary<int, ILinearSystem> LinearSystems { get; }
		public IGlobalLinearSystem LinearSystem { get; }

		public ISolverLogger Logger { get; }

		public string Name => name;

		public bool StartIterativeSolverFromPreviousSolution { get; set; } = false;

		public virtual Dictionary<int, IMatrix> BuildGlobalMatrices(IElementMatrixProvider elementMatrixProvider, 
			Func<int, bool> mustUpdateSubdomain)
		{
			throw new NotImplementedException();
			////TODOMPI: This must be called after ISolver.Initialize()
			//Func<int, IMatrix> buildKff = subdomainID =>
			//{
			//	Debug.WriteLine($"Subdomain {subdomainID} will try to build Kff");
			//	if (mustUpdateSubdomain(subdomainID))
			//	{
			//		ISubdomain subdomain = model.GetSubdomain(subdomainID);
			//		matrixManagersBasic[subdomainID].BuildKff(
			//			subdomain.FreeDofOrdering, subdomain.Elements, elementMatrixProvider);
			//	}
			//	return (IMatrix)LinearSystems[subdomainID].Matrix;
			//};
			//Dictionary<int, IMatrix> matricesKff = environment.CreateDictionaryPerNode(buildKff);
			//stiffnessDistribution.CalcSubdomainScaling(indexer);

			//return matricesKff;
		}

		public virtual Dictionary<int, (IMatrix matrixFreeFree, IMatrixView matrixFreeConstr, IMatrixView matrixConstrFree,
			IMatrixView matrixConstrConstr)> BuildGlobalSubmatrices(IElementMatrixProvider elementMatrixProvider)
		{
			throw new NotImplementedException();
		}

		public virtual Dictionary<int, SparseVector> DistributeNodalLoads(Table<INode, IDofType, double> nodalLoads)
			=> stiffnessDistribution.DistributeNodalLoads(nodalLoads);

		public virtual void HandleMatrixWillBeSet()
		{
		}

		public virtual void Initialize()
		{
			// Reordering the internal dofs is not done here, since subdomain Kff must be built first. 
			environment.DoPerNode(subdomainID => 
				dofManagerPsm.GetSubdomainDofs(subdomainID).SeparateFreeDofsIntoBoundaryAndInternal()); 
			dofManagerPsm.FindCommonDofsBetweenSubdomains();
			this.indexer = dofManagerPsm.CreateDistributedVectorIndexer();

			//TODOMPI: What should I log here? And where? There is not a central place for logs.
			//Logger.LogNumDofs("Global boundary dofs", dofSeparatorPsm.GetNumBoundaryDofsCluster(clusterID));
		}

		public virtual Dictionary<int, Matrix> InverseSystemMatrixTimesOtherMatrix(Dictionary<int, IMatrixView> otherMatrix)
		{
			throw new NotImplementedException();
		}

		public virtual void OrderDofs(bool alsoOrderConstrainedDofs)
		{
			throw new NotImplementedException();
			//environment.DoPerNode(subdomainID =>
			//{
			//	ISubdomain subdomain = model.GetSubdomain(subdomainID);
			//	subdomain.FreeDofOrdering = dofOrderer.OrderFreeDofs(subdomain);
			//});
		}

		public virtual void PreventFromOverwrittingSystemMatrices()
		{
		}

		public virtual void Solve()
		{
			Action<int> calcSubdomainMatrices = subdomainID =>
			{
				//TODO: This should only happen if the connectivity of the subdomain changes. 
				matrixManagersPsm[subdomainID].ReorderInternalDofs();

				//TODO: These should happen if the connectivity or stiffness of the subdomain changes
				matrixManagersPsm[subdomainID].ExtractKiiKbbKib();
				matrixManagersPsm[subdomainID].InvertKii();

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
			Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
			Debug.WriteLine("Iterations for boundary problem = " + stats.NumIterationsRequired);
		}

		public class Builder
		{
			private readonly IComputeEnvironment environment;

			public Builder(IComputeEnvironment environment)
			{
				DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering(), true);
				ExplicitSubdomainMatrices = false;

				//TODO: perhaps use a custom convergence check like in FETI
				var pcgBuilder = new PcgAlgorithm.Builder();
				pcgBuilder.ResidualTolerance = 1E-6;
				pcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(100);
				InterfaceProblemSolver = pcgBuilder.Build();
				IsHomogeneousProblem = true;

				MatrixManagerFactory = new PsmSubdomainMatrixManagerSymmetricCSparse.Factory();
				Preconditioner = new PsmPreconditionerIdentity();
				this.environment = environment;
			}

			public IDofOrderer DofOrderer { get; set; }

			public bool ExplicitSubdomainMatrices { get; set; }

			public IDistributedIterativeMethod InterfaceProblemSolver { get; set; }

			public bool IsHomogeneousProblem { get; set; }

			public IPsmSubdomainMatrixManagerFactory MatrixManagerFactory { get; set; }

			public IPsmPreconditioner Preconditioner { get; set; }

			public PsmSolver BuildSolver(IModel model, SubdomainTopology subdomainTopology)
			{
				return new PsmSolver(environment, model, subdomainTopology, DofOrderer, MatrixManagerFactory, 
					ExplicitSubdomainMatrices, Preconditioner, InterfaceProblemSolver, IsHomogeneousProblem);
			}
		}
	}
}
