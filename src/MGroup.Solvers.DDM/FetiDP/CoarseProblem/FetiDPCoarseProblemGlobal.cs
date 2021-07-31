using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemGlobal : IFetiDPCoarseProblem
	{
		private readonly IComputeEnvironment environment;
		private readonly FetiDPCoarseProblemGlobalDofs coarseProblemDofs;
		private readonly IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix;
		private readonly FetiDPCoarseProblemGlobalSolver coarseProblemSolver;
		private readonly Func<int, FetiDPSubdomainDofs> getSubdomainDofs;
		private readonly Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices;

		public FetiDPCoarseProblemGlobal(IComputeEnvironment environment, IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix,
			Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices)
		{
			this.environment = environment;
			this.coarseProblemMatrix = coarseProblemMatrix;
			this.getSubdomainDofs = getSubdomainDofs;
			this.getSubdomainMatrices = getSubdomainMatrices;
			this.coarseProblemDofs = new FetiDPCoarseProblemGlobalDofs();
			this.coarseProblemSolver = new FetiDPCoarseProblemGlobalSolver(coarseProblemDofs, coarseProblemMatrix);
		}

		public virtual void FindCoarseProblemDofs()
		{
			environment.DoGlobalOperation(
				getLocalNodeInput: s => getSubdomainDofs(s).DofOrderingCorner,
				globalOperation: subdomainCornerDofsInGlobalMem =>
				{
					coarseProblemDofs.FindGlobalCornerDofs(subdomainCornerDofsInGlobalMem);
					coarseProblemDofs.CalcSubdomainGlobalCornerDofMaps();
					DofPermutation permutation = coarseProblemMatrix.ReorderGlobalCornerDofs(
					coarseProblemDofs.NumGlobalCornerDofs, coarseProblemDofs.SubdomainToGlobalCornerDofs);
					coarseProblemDofs.ReorderGlobalCornerDofs(permutation);
				});

			//TODOMPI: delete these
			//Dictionary<int, IntDofTable> subdomainCornerDofs =
			//	environment.TransferNodeDataToGlobalMemory(s => getSubdomainDofs(s).DofOrderingCorner);
			//environment.DoGlobalOperation(() =>
			//{
			//	coarseProblemDofs.FindGlobalCornerDofs(subdomainCornerDofs);
			//	coarseProblemDofs.CalcSubdomainGlobalCornerDofMaps();
			//	DofPermutation permutation = coarseProblemMatrix.ReorderGlobalCornerDofs(
			//	coarseProblemDofs.NumGlobalCornerDofs, coarseProblemDofs.SubdomainToGlobalCornerDofs);
			//	coarseProblemDofs.ReorderGlobalCornerDofs(permutation);
			//});
		}

		public virtual void PrepareMatricesForSolution()
		{
			environment.DoPerNode(subdomainID => getSubdomainMatrices(subdomainID).CalcSchurComplementOfRemainderDofs());

			environment.DoGlobalOperation(
				getLocalNodeInput: s => getSubdomainMatrices(s).SchurComplementOfRemainderDofs,
				globalOperation: subdomainMatricesSccInGlobalMem =>
				{
					coarseProblemMatrix.InvertGlobalScc(coarseProblemDofs.NumGlobalCornerDofs,
						coarseProblemDofs.SubdomainToGlobalCornerDofs, subdomainMatricesSccInGlobalMem);
				});

			//TODOMPI: delete these
			//Dictionary<int, IMatrix> subdomainMatricesScc =
			//	environment.TransferNodeDataToGlobalMemory(s => getSubdomainMatrices(s).SchurComplementOfRemainderDofs);
			//environment.DoGlobalOperation(() =>
			//{
			//	coarseProblemMatrix.InvertGlobalScc(
			//		coarseProblemDofs.NumGlobalCornerDofs, coarseProblemDofs.SubdomainToGlobalCornerDofs, subdomainMatricesScc);
			//});
		}

		public virtual void SolveCoarseProblem(
			IDictionary<int, Vector> coarseProblemRhs, IDictionary<int, Vector> coarseProblemSolution)
		{
			Dictionary<int, Vector> subdomainSolutionVectorsLocal = environment.DoGlobalOperation(
				getLocalNodeInput: s => coarseProblemRhs[s],
				globalOperation: subdomainRhsVectorsInGlobalMem =>
				{
					//TODO: These should be cached and cleared somehow, as should the vectors in coarseProblemSolution
					var subdomainSolutionVectorsGlobal = new Dictionary<int, Vector>();
					coarseProblemSolver.SolveCoarseProblem(subdomainRhsVectorsInGlobalMem, subdomainSolutionVectorsGlobal);
					return subdomainSolutionVectorsGlobal;
				});

			//TODOMPI: delete these
			//Dictionary<int, Vector> subdomainRhsVectors = environment.TransferNodeDataToGlobalMemory(s => coarseProblemRhs[s]);
			//Dictionary<int, Vector> subdomainSolutionVectorsGlobal = null;
			//environment.DoGlobalOperation(() =>
			//{
			//	//TODO: These should be cached and cleared somehow, as should the vectors in coarseProblemSolution
			//	subdomainSolutionVectorsGlobal = new Dictionary<int, Vector>(); 
			//	coarseProblemSolver.SolveCoarseProblem(subdomainRhsVectors, subdomainSolutionVectorsGlobal);
			//});
			//Dictionary<int, Vector> subdomainSolutionVectorsLocal = 
			//	environment.TransferNodeDataToLocalMemories(subdomainSolutionVectorsGlobal);

			environment.DoPerNode(s => coarseProblemSolution[s].CopyFrom(subdomainSolutionVectorsLocal[s]));
		}

		public class Factory : IFetiDPCoarseProblemFactory
		{
			private readonly IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix;

			public Factory(IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix)
			{
				this.coarseProblemMatrix = coarseProblemMatrix;
			}

			public IFetiDPCoarseProblem CreateCoarseProblem(
				IComputeEnvironment environment, SubdomainTopology subdomainTopology,
				Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices)
			{
				return new FetiDPCoarseProblemGlobal(environment, coarseProblemMatrix, getSubdomainDofs, getSubdomainMatrices);
			}
		}
	}
}
