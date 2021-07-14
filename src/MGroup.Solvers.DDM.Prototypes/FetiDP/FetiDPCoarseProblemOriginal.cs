using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;

namespace MGroup.Solvers.DDM.Prototypes.FetiDP
{
	public class FetiDPCoarseProblemOriginal : IFetiDPCoarseProblem
	{
		private readonly FetiDPSubdomainDofs dofs;
		private readonly IModel model;
		private readonly FetiDPSubdomainStiffnesses stiffnesses;

		public FetiDPCoarseProblemOriginal(IModel model, FetiDPSubdomainDofs dofs, 
			FetiDPSubdomainStiffnesses stiffnesses)
		{
			this.model = model;
			this.dofs = dofs;
			this.stiffnesses = stiffnesses;
		}

		public DofTable GlobalDofOrderingCorner { get; set; }

		public Matrix InvScc { get; set; }

		public BlockMatrix MatrixLce { get; set; }

		public int NumGlobalDofsCorner { get; set; }

		public Dictionary<int, Matrix> SubdomainMatricesLc { get; } = new Dictionary<int, Matrix>();

		public void FindDofs()
		{
			FindGlobalCornerDofs();
			MapCornerDofsGlobalToSubdomains();
			CalcLce();
		}

		public void Prepare()
		{
			var globalScc = Matrix.CreateZero(NumGlobalDofsCorner, NumGlobalDofsCorner);
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				Matrix Lc = SubdomainMatricesLc[s];
				Matrix localScc = stiffnesses.Scc[s];
				globalScc.AddIntoThis(Lc.Transpose() * localScc * Lc);
			}
			this.InvScc = globalScc.Invert();
		}

		private void CalcLce()
		{
			MatrixLce = BlockMatrix.CreateCol(dofs.NumSubdomainDofsCorner, NumGlobalDofsCorner);
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				MatrixLce.AddBlock(s, 0, SubdomainMatricesLc[s]);
			}
		}

		private void FindGlobalCornerDofs()
		{
			var globalCornerDofs = new SortedDofTable();
			int numCornerDofs = 0;
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				foreach ((INode node, IDofType dof, int idx) in dofs.SubdomainDofOrderingCorner[subdomain.ID])
				{
					bool didNotExist = globalCornerDofs.TryAdd(node.ID, AllDofs.GetIdOfDof(dof), numCornerDofs);
					if (didNotExist)
					{
						numCornerDofs++;
					}
				}
			}

			var cornerDofOrdering = new DofTable();
			foreach ((int nodeID, int dofID, int idx) in globalCornerDofs)
			{
				cornerDofOrdering[model.GetNode(nodeID), AllDofs.GetDofWithId(dofID)] = idx;
			}

			GlobalDofOrderingCorner = cornerDofOrdering;
			NumGlobalDofsCorner = numCornerDofs;
		}

		private void MapCornerDofsGlobalToSubdomains()
		{
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				DofTable subdomainDofs = dofs.SubdomainDofOrderingCorner[subdomain.ID];
				var Lc = Matrix.CreateZero(dofs.NumSubdomainDofsCorner[subdomain.ID], NumGlobalDofsCorner);
				foreach ((INode node, IDofType dof, int subdomainIdx) in subdomainDofs)
				{
					int globalIdx = GlobalDofOrderingCorner[node, dof];
					Lc[subdomainIdx, globalIdx] = 1.0;
				}
				SubdomainMatricesLc[subdomain.ID] = Lc;
			}
		}
	}
}
