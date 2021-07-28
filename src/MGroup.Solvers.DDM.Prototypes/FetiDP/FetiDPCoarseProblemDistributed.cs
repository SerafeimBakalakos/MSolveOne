using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Iterative.Termination;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;
using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.Solvers.DDM.Prototypes.FetiDP
{
	public class FetiDPCoarseProblemDistributed : IFetiDPCoarseProblem
	{
		private readonly FetiDPSubdomainDofs dofs;
		private readonly IModel model;
		private readonly FetiDPSubdomainStiffnesses stiffnesses;

		private IVectorMultipliable coarseProblemMatrix;

		public FetiDPCoarseProblemDistributed(IModel model, FetiDPSubdomainDofs dofs, 
			FetiDPSubdomainStiffnesses stiffnesses, double pcgResidualTolerance)
		{
			this.model = model;
			this.dofs = dofs;
			this.stiffnesses = stiffnesses;

			var iterativeSolverBuilder = new PcgAlgorithm.Builder();
			iterativeSolverBuilder.MaxIterationsProvider = new PercentageMaxIterationsProvider(1.0);
			iterativeSolverBuilder.ResidualTolerance = 0.01 * pcgResidualTolerance; // HERE: Let user define this.
			IterativeSolver = iterativeSolverBuilder.Build();
		}

		public PcgAlgorithm IterativeSolver { get; }

		public IterativeStatistics IterativeSolverStats { get; set; }

		public BlockMatrix MatrixMce { get; set; }

		public Dictionary<int, int[]> SubdomainCornerDofMultiplicities { get; } = new Dictionary<int, int[]>();

		public Dictionary<int, Dictionary<int, Matrix>> SubdomainMatricesMc { get; } 
			= new Dictionary<int, Dictionary<int, Matrix>>();

		public void FindDofs()
		{
			MatrixMce = BlockMatrix.Create(dofs.NumSubdomainDofsCorner, dofs.NumSubdomainDofsCorner);
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				SubdomainCornerDofMultiplicities[subdomain.ID] = FindCornerDofMultiplicities(subdomain);

				var subdomainMc = new Dictionary<int, Matrix>();
				SubdomainMatricesMc[subdomain.ID] = subdomainMc;
				foreach (ISubdomain other in model.EnumerateSubdomains())
				{
					Matrix Mc = MapInterSubdomainDofs(subdomain, other);
					subdomainMc[other.ID] = Mc;
					MatrixMce.AddBlock(subdomain.ID, other.ID, Mc);
				}
			}

			int[][] multiplicities = CalcMultiplicities();
			MatrixMce.RowMultiplicities = multiplicities;
			MatrixMce.ColMultiplicities = multiplicities;
		}

		public void Prepare()
		{
			coarseProblemMatrix = new MatrixProduct(MatrixMce, stiffnesses.Scce);
			IterativeSolver.Clear();
			IterativeSolverStats = null;
		}

		public IVector Solve(IVector expandedDomainRhs)
		{
			var preconditioner = new IdentityPreconditioner();
			IVector coarseProblemRhs = MatrixMce.Multiply(expandedDomainRhs);
			IVector coarseProblemSolution = coarseProblemRhs.CreateZeroVectorWithSameFormat();
			IterativeSolverStats = IterativeSolver.Solve(coarseProblemMatrix, preconditioner, coarseProblemRhs,
				coarseProblemSolution, true, coarseProblemRhs.CreateZeroVectorWithSameFormat);
			Debug.WriteLine($"Coarse problem PCG: num iterations = {IterativeSolverStats.NumIterationsRequired}, " +
				$"residual norm ratio = {IterativeSolverStats.ResidualNormRatioEstimation}");
			return coarseProblemSolution;
		}


		private int[] FindCornerDofMultiplicities(ISubdomain subdomain)
		{
			var result = new int[dofs.NumSubdomainDofsCorner[subdomain.ID]];
			foreach ((int node, int dof, int idx) in dofs.SubdomainDofOrderingCorner[subdomain.ID])
			{
				result[idx] = model.GetNode(node).Subdomains.Count;
			}
			return result;
		}

		private Matrix MapInterSubdomainDofs(ISubdomain rowSubdomain, ISubdomain colSubdomain)
		{
			int sR = rowSubdomain.ID;
			int sC = colSubdomain.ID;
			var result = Matrix.CreateZero(dofs.NumSubdomainDofsCorner[sR], dofs.NumSubdomainDofsCorner[sC]);
			IntDofTable rowCornerDofs = dofs.SubdomainDofOrderingCorner[sR];
			IntDofTable colCornerDofs = dofs.SubdomainDofOrderingCorner[sC];
			foreach ((int node, int dof, int row) in rowCornerDofs)
			{
				bool isCommonDof = colCornerDofs.TryGetValue(node, dof, out int col);
				if (isCommonDof)
				{
					result[row, col] = 1.0;
				}
			}
			return result;
		}

		public int[][] CalcMultiplicities()
		{
			var multiplicities = new int[model.NumSubdomains][];
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				multiplicities[s] = SubdomainCornerDofMultiplicities[s];
			}
			return multiplicities;
		}

		
	}
}
