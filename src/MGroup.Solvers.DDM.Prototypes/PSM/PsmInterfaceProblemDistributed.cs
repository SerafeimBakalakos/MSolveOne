using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Matrices;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
	public class PsmInterfaceProblemDistributed : IPsmInterfaceProblem
	{
		private readonly IModel model;
		private readonly PsmSubdomainDofs dofs;

		public PsmInterfaceProblemDistributed(IModel model, PsmSubdomainDofs dofs)
		{
			this.model = model;
			this.dofs = dofs;
		}

		public BlockMatrix MatrixMbe { get; set; }

		public Dictionary<int, int[]> SubdomainBoundaryDofMultiplicities { get; } = new Dictionary<int, int[]>();

		public Dictionary<int, Dictionary<int, Matrix>> SubdomainMatricesMb { get; }
			= new Dictionary<int, Dictionary<int, Matrix>>();

		public void FindDofs()
		{
			MatrixMbe = BlockMatrix.Create(dofs.NumSubdomainDofsBoundary, dofs.NumSubdomainDofsBoundary);
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				SubdomainBoundaryDofMultiplicities[subdomain.ID] = FindBoundaryDofMultiplicities(subdomain);

				var subdomainMb = new Dictionary<int, Matrix>();
				SubdomainMatricesMb[subdomain.ID] = subdomainMb;
				foreach (ISubdomain other in model.EnumerateSubdomains())
				{
					Matrix Mb = MapInterSubdomainDofs(subdomain, other);
					subdomainMb[other.ID] = Mb;
					MatrixMbe.AddBlock(subdomain.ID, other.ID, Mb);
				}
			}

			int[][] multiplicities = CalcMultiplicities();
			MatrixMbe.RowMultiplicities = multiplicities;
			MatrixMbe.ColMultiplicities = multiplicities;
		}

		public IterativeStatistics Solve(PcgAlgorithm iterativeSolver, IPreconditioner preconditioner,
			BlockMatrix expandedDomainMatrix, BlockVector expandedDomainRhs, BlockVector expandedDomainSolution)
		{
			int[][] multiplicities = MatrixMbe.ColMultiplicities;
			expandedDomainMatrix.RowMultiplicities = multiplicities;
			expandedDomainMatrix.ColMultiplicities = multiplicities;
			expandedDomainRhs.Multiplicities = multiplicities;
			expandedDomainSolution.Multiplicities = multiplicities;

			// Interface problem
			var interfaceMatrix = new MatrixProduct(MatrixMbe, expandedDomainMatrix);
			BlockVector interfaceRhs = MatrixMbe * expandedDomainRhs;

			// Interface problem solution using CG
			var stats = iterativeSolver.Solve(interfaceMatrix, preconditioner, interfaceRhs, expandedDomainSolution, false,
				expandedDomainSolution.CreateZeroVectorWithSameFormat);

			return stats;
		}

		private int[] FindBoundaryDofMultiplicities(ISubdomain subdomain)
		{
			var result = new int[dofs.NumSubdomainDofsBoundary[subdomain.ID]];
			foreach ((int node, int dof, int idx) in dofs.SubdomainDofOrderingBoundary[subdomain.ID])
			{
				result[idx] = model.GetNode(node).Subdomains.Count;
			}
			return result;
		}

		private Matrix MapInterSubdomainDofs(ISubdomain rowSubdomain, ISubdomain colSubdomain)
		{
			int sR = rowSubdomain.ID;
			int sC = colSubdomain.ID;
			var result = Matrix.CreateZero(dofs.NumSubdomainDofsBoundary[sR], dofs.NumSubdomainDofsBoundary[sC]);
			IntDofTable rowBoundaryDofs = dofs.SubdomainDofOrderingBoundary[sR];
			IntDofTable colBoundaryDofs = dofs.SubdomainDofOrderingBoundary[sC];
			foreach ((int node, int dof, int row) in rowBoundaryDofs)
			{
				bool isCommonDof = colBoundaryDofs.TryGetValue(node, dof, out int col);
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
				multiplicities[s] = SubdomainBoundaryDofMultiplicities[s];
			}
			return multiplicities;
		}
	}
}
