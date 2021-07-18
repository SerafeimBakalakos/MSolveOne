using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
	public class PsmInterfaceProblemOriginal : IPsmInterfaceProblem
	{
		private readonly IModel model;
		private readonly PsmSubdomainDofs dofs;

		public PsmInterfaceProblemOriginal(IModel model, PsmSubdomainDofs dofs)
		{
			this.model = model;
			this.dofs = dofs;
		}

		public DofTable GlobalDofOrderingBoundary { get; set; }

		public BlockMatrix MatrixLbe { get; set; }

		public int NumGlobalDofsBoundary { get; set; }

		public Dictionary<int, Matrix> SubdomainMatricesLb { get; } = new Dictionary<int, Matrix>();

		public void FindDofs()
		{
			FindGlobalBoundaryDofs();
			MapBoundaryDofsGlobalToSubdomains();
			CalcLbe();
		}

		public IterativeStatistics Solve(PcgAlgorithm iterativeSolver, IPreconditioner preconditioner,
			BlockMatrix expandedDomainMatrix, BlockVector expandedDomainRhs, BlockVector expandedDomainSolution)
		{
			// Interface problem
			Matrix fullLbe = MatrixLbe.CopyToFullMatrix();
			Matrix fullSbbe = expandedDomainMatrix.CopyToFullMatrix();
			Vector fullFbeCond = expandedDomainRhs.CopyToFullVector();
			Matrix interfaceMatrix = fullLbe.Transpose() * fullSbbe * fullLbe;
			Vector interfaceRhs = fullLbe.Transpose() * fullFbeCond;
			var interfaceSolution = Vector.CreateZero(interfaceRhs.Length);

			// Interface problem solution using CG
			var pcgBuilder = new PcgAlgorithm.Builder();
			var stats = iterativeSolver.Solve(interfaceMatrix, preconditioner, interfaceRhs, interfaceSolution, false,
				() => Vector.CreateZero(interfaceRhs.Length));

			expandedDomainSolution.CopyFrom(MatrixLbe * interfaceSolution);

			return stats;
		}

		private void CalcLbe()
		{
			MatrixLbe = BlockMatrix.CreateCol(dofs.NumSubdomainDofsBoundary, NumGlobalDofsBoundary);
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				MatrixLbe.AddBlock(s, 0, SubdomainMatricesLb[s]);
			}
		}

		private void FindGlobalBoundaryDofs()
		{
			var globalBoundaryDofs = new SortedDofTable();
			int numBoundaryDofs = 0;
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				foreach ((INode node, IDofType dof, int idx) in dofs.SubdomainDofOrderingBoundary[subdomain.ID])
				{
					bool didNotExist = globalBoundaryDofs.TryAdd(node.ID, model.AllDofs.GetIdOfDof(dof), numBoundaryDofs);
					if (didNotExist)
					{
						numBoundaryDofs++;
					}
				}
			}

			var boundaryDofOrdering = new DofTable();
			foreach ((int nodeID, int dofID, int idx) in globalBoundaryDofs)
			{
				boundaryDofOrdering[model.GetNode(nodeID), model.AllDofs.GetDofWithId(dofID)] = idx;
			}

			GlobalDofOrderingBoundary = boundaryDofOrdering;
			NumGlobalDofsBoundary = numBoundaryDofs;
		}

		private void MapBoundaryDofsGlobalToSubdomains()
		{
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				DofTable subdomainDofs = dofs.SubdomainDofOrderingBoundary[subdomain.ID];
				var Lb = Matrix.CreateZero(dofs.NumSubdomainDofsBoundary[subdomain.ID], NumGlobalDofsBoundary);
				foreach ((INode node, IDofType dof, int subdomainIdx) in subdomainDofs)
				{
					int globalIdx = GlobalDofOrderingBoundary[node, dof];
					Lb[subdomainIdx, globalIdx] = 1.0;
				}
				SubdomainMatricesLb[subdomain.ID] = Lb;
			}
		}
	}
}
