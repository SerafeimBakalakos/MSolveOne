using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.IterativeMethods;
using MGroup.LinearAlgebra.Distributed.IterativeMethods.PCG;
using MGroup.LinearAlgebra.Distributed.IterativeMethods.PCG.Reorthogonalization;
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
		private readonly IFetiDPCoarseProblemDistributedPreconditioner coarseProblemPreconditioner;
		private readonly IDistributedIterativeMethod coarseProblemSolver;
		private readonly bool areSchurComplementsExplicit;

		private DistributedOverlappingIndexer cornerDofIndexer;
		private DistributedOverlappingTransformation coarseProblemMatrix;

		public FetiDPCoarseProblemDistributed(IComputeEnvironment environment, SubdomainTopology subdomainTopology,
			Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices,
			IDistributedIterativeMethod coarseProblemSolver, bool useJacobiPreconditioner, bool areSchurComplementsExplicit)
		{
			this.environment = environment;
			this.subdomainTopology = subdomainTopology;
			this.getSubdomainDofs = getSubdomainDofs;
			this.getSubdomainMatrices = getSubdomainMatrices;
			this.coarseProblemSolver = coarseProblemSolver;
			this.areSchurComplementsExplicit = areSchurComplementsExplicit;

			if (useJacobiPreconditioner)
			{
				coarseProblemPreconditioner = new FetiDPCoarseProblemDistributedPreconditionerJacobi(
					environment, getSubdomainMatrices);
				this.areSchurComplementsExplicit = true;
			}
			else
			{
				coarseProblemPreconditioner = new FetiDPCoarseProblemDistributedPreconditionerIdentity();
			}
		}

		public void FindCoarseProblemDofs()
		{
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
			coarseProblemPreconditioner.Calculate(cornerDofIndexer);
		}

		public void SolveCoarseProblem(IDictionary<int, Vector> coarseProblemRhs, IDictionary<int, Vector> coarseProblemSolution)
		{
			var distributedRhs = new DistributedOverlappingVector(cornerDofIndexer, coarseProblemRhs);
			var distributedSolution = new DistributedOverlappingVector(cornerDofIndexer, coarseProblemSolution);

			distributedRhs.SumOverlappingEntries();


			#region debug
			//// First rhs
			//var b0 = new DistributedOverlappingVector(cornerDofIndexer);
			//var x0 = new DistributedOverlappingVector(cornerDofIndexer);
			//foreach (int s in b0.LocalVectors.Keys)
			//{
			//	int[] multiplicities = cornerDofIndexer.GetLocalComponent(s).Multiplicities;
			//	Vector v = b0.LocalVectors[s];
			//	for (int i = 0; i < multiplicities.Length; ++i)
			//	{
			//		v[i] = multiplicities[i];
			//	}
			//}
			//IterativeStatistics stats0 = coarseProblemSolver.Solve(
			//	coarseProblemMatrix, coarseProblemPreconditioner.Preconditioner, b0, x0, true);

			//// Second rhs
			//double dx = 0.01;
			//double a = 1.0;
			//var b1 = new DistributedOverlappingVector(cornerDofIndexer);
			//var x1 = new DistributedOverlappingVector(cornerDofIndexer);
			//foreach (int s in b1.LocalVectors.Keys)
			//{
			//	int[] multiplicities = cornerDofIndexer.GetLocalComponent(s).Multiplicities;
			//	Vector v = b1.LocalVectors[s];
			//	for (int i = 0; i < multiplicities.Length; ++i)
			//	{
			//		v[i] = a * multiplicities[i] + dx;
			//	}
			//}

			//Debug.WriteLine("Starting 2nd rhs");

			//IterativeStatistics stats1 = coarseProblemSolver.Solve(
			//	coarseProblemMatrix, coarseProblemPreconditioner.Preconditioner, b1, x1, true);
			#endregion

			if (coarseProblemSolver is PcgAlgorithmBase pcg)
			{
				//pcg.ResidualTolerance = 1E-20;
				//pcgReortho.ReorthoCache.RemoveOldDirectionVectorData(5);
				//pcgReortho.ReorthoCache.RemoveNewDirectionVectorData(5);
			}

			IterativeStatistics stats = coarseProblemSolver.Solve(
				coarseProblemMatrix, coarseProblemPreconditioner.Preconditioner, distributedRhs, distributedSolution, true);

			#region debug
			if (coarseProblemSolver is ReorthogonalizedPcg pcgReortho)
			{
				//pcgReortho.ReorthoCache.Clear();
				//pcgReortho.ReorthoCache.RemoveOldDirectionVectorData(5);
				//pcgReortho.ReorthoCache.RemoveNewDirectionVectorData(5);
			}
			#endregion

			Debug.WriteLine($"Coarse problem solution: iterations = {stats.NumIterationsRequired}, " +
				$"residual norm ratio = {stats.ResidualNormRatioEstimation}");
		}

		public class Factory : IFetiDPCoarseProblemFactory
		{
			public Factory()
			{
				AreSchurComplementsExplicit = true;
				UseJacobiPreconditioner = true;

				var pcgBuilder = new PcgAlgorithm.Builder();
				pcgBuilder.ResidualTolerance = 1E-6;
				pcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(100);
				CoarseProblemSolver = pcgBuilder.Build();
			}

			public bool AreSchurComplementsExplicit { get; set; }

			public IDistributedIterativeMethod CoarseProblemSolver { get; set; }

			public bool UseJacobiPreconditioner { get; set; }

			public IFetiDPCoarseProblem CreateCoarseProblem(
				IComputeEnvironment environment, SubdomainTopology subdomainTopology, 
				Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices)
			{
				return new FetiDPCoarseProblemDistributed(environment, subdomainTopology, getSubdomainDofs,
					getSubdomainMatrices, CoarseProblemSolver, UseJacobiPreconditioner, AreSchurComplementsExplicit);
			}
		}
	}
}
