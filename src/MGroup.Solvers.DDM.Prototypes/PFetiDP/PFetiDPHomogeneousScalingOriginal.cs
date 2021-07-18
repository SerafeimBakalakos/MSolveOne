using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;
using MGroup.Solvers.DDM.Prototypes.PSM;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	public class PFetiDPHomogeneousScalingOriginal : IPFetiDPScaling
	{
		protected PsmInterfaceProblemOriginal interfaceProblem;
		protected PFetiDPSubdomainDofs pfetiDPDofs;

		public Matrix MatrixLpre { get; set;  }

		private Matrix MatrixWb { get; set; }

		public virtual void Initialize()
		{
			CalcWb();
			CalcLpre();
		}

		public virtual void InitializeComponents(PFetiDPSolver solver)
		{
			this.interfaceProblem = (PsmInterfaceProblemOriginal)(solver.InterfaceProblem);
			this.pfetiDPDofs = solver.PFetiDPDofs;
		}

		private void CalcLpre()
		{
			Matrix Lbe = interfaceProblem.MatrixLbe.CopyToFullMatrix();
			Matrix Nrbe = pfetiDPDofs.MatrixNrbe.CopyToFullMatrix();
			Matrix Lpbe = Lbe * MatrixWb;
			MatrixLpre = Nrbe * Lpbe;
		}

		private void CalcWb()
		{
			MatrixWb = Matrix.CreateZero(interfaceProblem.NumGlobalDofsBoundary, interfaceProblem.NumGlobalDofsBoundary);
			foreach ((INode node, _, int idx) in interfaceProblem.GlobalDofOrderingBoundary)
			{
				MatrixWb[idx, idx] = 1.0 / node.Subdomains.Count;
			}
		}
	}
}
