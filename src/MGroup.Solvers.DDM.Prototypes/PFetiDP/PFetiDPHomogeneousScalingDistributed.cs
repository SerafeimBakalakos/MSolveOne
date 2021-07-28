using System;
using System.Collections.Generic;
using System.Text;
using DotNumerics.ODE.DVode;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Matrices;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;
using MGroup.Solvers.DDM.Prototypes.PSM;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	/// <summary>
	/// Lpbe = Wbe * Lbe, instead of the original Lpbe = Lbe * Wb
	/// </summary>
	public class PFetiDPHomogeneousScalingDistributed : IPFetiDPScaling
	{
		private readonly IModel model;
		private PsmSubdomainDofs psmDofs;

		public PFetiDPHomogeneousScalingDistributed(IModel model)
		{
			this.model = model;
		}

		public BlockMatrix MatrixWbe { get; set; }

		public Dictionary<int, Matrix> SudomainMatricesWb { get; } = new Dictionary<int, Matrix>();

		public void Initialize()
		{
			this.MatrixWbe = BlockMatrix.Create(psmDofs.NumSubdomainDofsBoundary, psmDofs.NumSubdomainDofsBoundary);
			foreach (int s in psmDofs.SubdomainDofOrderingBoundary.Keys)
			{
				int nbs = psmDofs.NumSubdomainDofsBoundary[s];
				var Ws = Matrix.CreateZero(nbs, nbs);
				IntDofTable boundaryDofs = psmDofs.SubdomainDofOrderingBoundary[s];
				foreach ((int node, _, int idx) in boundaryDofs)
				{
					Ws[idx, idx] = 1.0 / model.GetNode(node).Subdomains.Count;
				}
				this.SudomainMatricesWb[s] = Ws;
				this.MatrixWbe.AddBlock(s, s, Ws);
			}
		}

		public void InitializeComponents(PFetiDPSolver solver)
		{
			this.psmDofs = solver.PsmDofs;
		}
	}
}
