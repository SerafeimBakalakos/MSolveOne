using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Matrices;
using MGroup.Solvers.DDM.Prototypes.FetiDP;
using MGroup.Solvers.DDM.Prototypes.PSM;
using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	public class InterfaceCoarseProblemBridgeOriginal : IInterfaceCoarseProblemBridge
	{
		private PsmInterfaceProblemOriginal interfaceProblem;
		private FetiDPCoarseProblemOriginal coarseProblem;

		public Matrix GlobalMatrixNcb { get; set; }

		public void InitializeComponents(PFetiDPSolver solver)
		{
			this.interfaceProblem = (PsmInterfaceProblemOriginal)(solver.InterfaceProblem);
			this.coarseProblem = (FetiDPCoarseProblemOriginal)(solver.CoarseProblem);
		}

		public void LinkInterfaceCoarseProblems()
		{
			IntDofTable boundaryDofs = interfaceProblem.GlobalDofOrderingBoundary;
			IntDofTable cornerDofs = coarseProblem.GlobalDofOrderingCorner;
			GlobalMatrixNcb = Matrix.CreateZero(coarseProblem.NumGlobalDofsCorner, interfaceProblem.NumGlobalDofsBoundary);
			foreach ((int node, int dof, int c) in cornerDofs)
			{
				int b = boundaryDofs[node, dof];
				GlobalMatrixNcb[c, b] = 1.0;
			}
		}
	}
}
