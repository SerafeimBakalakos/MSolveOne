using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.IterativeMethods;
using MGroup.LinearAlgebra.Distributed.IterativeMethods.Preconditioning;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.Termination;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemDistributed : IFetiDPCoarseProblem
	{
		private readonly IComputeEnvironment environment;
		private readonly SubdomainTopology subdomainTopology;
		private readonly Func<int, FetiDPSubdomainDofs> getSubdomainDofs;
		private readonly Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices;
		private readonly IDistributedIterativeMethod coarseProblemSolver;
		private readonly bool areSchurComplementsExplicit;

		private DistributedOverlappingIndexer cornerDofIndexer;
		private DistributedOverlappingTransformation coarseProblemMatrix;

		public FetiDPCoarseProblemDistributed(IComputeEnvironment environment, SubdomainTopology subdomainTopology,
			Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices,
			IDistributedIterativeMethod coarseProblemSolver, bool areSchurComplementsExplicit)
		{
			this.environment = environment;
			this.subdomainTopology = subdomainTopology;
			this.getSubdomainDofs = getSubdomainDofs;
			this.getSubdomainMatrices = getSubdomainMatrices;
			this.coarseProblemSolver = coarseProblemSolver;
			this.areSchurComplementsExplicit = areSchurComplementsExplicit;
		}

		public void FindCoarseProblemDofs()
		{
			//HERE: there is some mistake in this.
			cornerDofIndexer = subdomainTopology.CreateDistributedVectorIndexer(s => getSubdomainDofs(s).DofOrderingCorner);
		}

		public void PrepareMatricesForSolution()
		{
			if (areSchurComplementsExplicit)
			{
				environment.DoPerNode(s => getSubdomainMatrices(s).CalcSchurComplementOfRemainderDofs());
				coarseProblemMatrix = new DistributedOverlappingTransformation(cornerDofIndexer,
					(s, vIn, vOut) => getSubdomainMatrices(s).SchurComplementOfRemainderDofs.MultiplyIntoResult(vIn, vOut));
			}
			else
			{
				coarseProblemMatrix = new DistributedOverlappingTransformation(cornerDofIndexer,
					(s, vIn, vOut) => getSubdomainMatrices(s).MultiplySchurComplementImplicitly(vIn, vOut));
			}
		}

		public void SolveCoarseProblem(IDictionary<int, Vector> coarseProblemRhs, IDictionary<int, Vector> coarseProblemSolution)
		{
			var distributedRhs = new DistributedOverlappingVector(cornerDofIndexer, coarseProblemRhs);
			var distributedSolution = new DistributedOverlappingVector(cornerDofIndexer, coarseProblemSolution);
			var preconditioner = new IdentityPreconditioner();

			distributedRhs.SumOverlappingEntries();
			IterativeStatistics stats = coarseProblemSolver.Solve(
				coarseProblemMatrix, preconditioner, distributedRhs, distributedSolution, true);
			Debug.WriteLine($"Coarse problem solution: iterations = {stats.NumIterationsRequired}, " +
				$"residual norm ratio = {stats.ResidualNormRatioEstimation}");
		}

		public class Factory : IFetiDPCoarseProblemFactory
		{
			public Factory()
			{
				AreSchurComplementsExplicit = true;

				var pcgBuilder = new PcgAlgorithm.Builder();
				pcgBuilder.ResidualTolerance = 1E-6;
				pcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(100);
				CoarseProblemSolver = pcgBuilder.Build();
			}

			public bool AreSchurComplementsExplicit { get; set; }

			public IDistributedIterativeMethod CoarseProblemSolver { get; set; }

			public IFetiDPCoarseProblem CreateCoarseProblem(
				IComputeEnvironment environment, SubdomainTopology subdomainTopology, 
				Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices)
			{
				return new FetiDPCoarseProblemDistributed(environment, subdomainTopology, getSubdomainDofs,
					getSubdomainMatrices, CoarseProblemSolver, AreSchurComplementsExplicit);
			}
		}
	}
}
