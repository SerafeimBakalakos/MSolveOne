using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

//TODOMPI: This base -> derived class design (Template Method) uses inheritance. Replace it with a design that uses composition.
//		However the top level classes must be the ones that handle the environment, parallelization & asynchronous logic.
//		This is important, since the classes that are strictly DDM logic must be reused in different ways, depending on how
//		parallelization and data locality is chosen at runtime. It is much more flexible to let the top level classes schedule
//		the DDM tasks, than it is to just use template method / composition to call methods for transferring data inbetween the
//		rest of the algorithm steps.
namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public abstract class FetiDPCoarseProblemGlobalBase : IFetiDPCoarseProblem
	{
		protected readonly IComputeEnvironment environment;
		protected readonly FetiDPCoarseProblemGlobalDofs coarseProblemDofs;
		protected readonly IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix;
		protected readonly FetiDPCoarseProblemGlobalSolver coarseProblemSolver;
		protected readonly Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices;

		protected FetiDPCoarseProblemGlobalBase(IComputeEnvironment environment,
			IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices)
		{
			this.environment = environment;
			this.coarseProblemMatrix = coarseProblemMatrix;
			this.getSubdomainMatrices = getSubdomainMatrices;
			this.coarseProblemDofs = new FetiDPCoarseProblemGlobalDofs();
			this.coarseProblemSolver = new FetiDPCoarseProblemGlobalSolver(coarseProblemDofs, coarseProblemMatrix);
		}

		public void FindCoarseProblemDofs()
		{
			Dictionary<int, IntDofTable> subdomainCornerDofs = GatherSubdomainCornerDofs();
			environment.DoMasterNode(() =>
			{
				coarseProblemDofs.FindGlobalCornerDofs(subdomainCornerDofs);
				coarseProblemDofs.CalcSubdomainGlobalCornerDofMaps();
				DofPermutation permutation = coarseProblemMatrix.ReorderGlobalCornerDofs(
				coarseProblemDofs.NumGlobalCornerDofs, coarseProblemDofs.SubdomainToGlobalCornerDofs);
				coarseProblemDofs.ReorderGlobalCornerDofs(permutation);
			});
		}

		public void PrepareMatricesForSolution()
		{
			environment.DoPerNode(subdomainID => getSubdomainMatrices(subdomainID).CalcSchurComplementOfRemainderDofs());
			Dictionary<int, IMatrix> subdomainMatricesScc = GatherSubdomainMatricesScc();
			environment.DoMasterNode(() =>
			{
				coarseProblemMatrix.InvertGlobalScc(
					coarseProblemDofs.NumGlobalCornerDofs, coarseProblemDofs.SubdomainToGlobalCornerDofs, subdomainMatricesScc);
			});
		}

		public void SolveCoarseProblem(IDictionary<int, Vector> coarseProblemRhs, IDictionary<int, Vector> coarseProblemSolution)
		{
			Dictionary<int, Vector> subdomainRhsVectors = GatherSubdomainVectors(coarseProblemRhs);
			var subdomainSolutionVectors = new Dictionary<int, Vector>(); //TODO: These should be cached and cleared somehow
			environment.DoMasterNode(() =>
			{
				coarseProblemSolver.SolveCoarseProblem(subdomainRhsVectors, subdomainSolutionVectors);
			});
			ScatterSubdomainVectors(subdomainSolutionVectors, coarseProblemSolution);
		}

		protected abstract Dictionary<int, IntDofTable> GatherSubdomainCornerDofs();

		protected abstract Dictionary<int, IMatrix> GatherSubdomainMatricesScc();

		protected abstract Dictionary<int, Vector> GatherSubdomainVectors(IDictionary<int, Vector> coarseProblemRhs);

		protected abstract void ScatterSubdomainVectors(
			Dictionary<int, Vector> localVectors, IDictionary<int, Vector> remoteVectors);
	}
}
