using System;
using System.Collections.Generic;
using System.Text;
using DotNumerics.ODE.DVode;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Matrices;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;
using MGroup.Solvers.DDM.Prototypes.PSM;
using MGroup.MSolve.Discretization.Dofs;

//TODO: Merge this with the original. Use a flag to choose which way to calculate Lpbe
namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	/// <summary>
	/// Lpbe = Wbe * Lbe, instead of the original Lpbe = Lbe * Wb
	/// </summary>
	public class PFetiDPHomogeneousScalingModified : PFetiDPHomogeneousScalingOriginal
	{
		private readonly IModel model;
		private PsmSubdomainDofs psmDofs;

		public PFetiDPHomogeneousScalingModified(IModel model) : base(model)
		{
			this.model = model;
		}

		private BlockMatrix MatrixWbe { get; set; }

		private Dictionary<int, Matrix> SudomainMatricesWb { get; } = new Dictionary<int, Matrix>();

		public override void Initialize()
		{
			CalcWb();
			CalcLpre();
		}

		public override void InitializeComponents(PFetiDPSolver solver)
		{
			base.InitializeComponents(solver);
			this.psmDofs = solver.PsmDofs;
		}

		private void CalcLpre()
		{
			Matrix Lbe = interfaceProblem.MatrixLbe.CopyToFullMatrix();
			Matrix Nrbe = pfetiDPDofs.MatrixNrbe.CopyToFullMatrix();
			Matrix Wbe = MatrixWbe.CopyToFullMatrix();
			Matrix Lpbe = Wbe * Lbe;
			MatrixLpre = Nrbe * Lpbe;
		}

		private void CalcWb()
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

	}
}
