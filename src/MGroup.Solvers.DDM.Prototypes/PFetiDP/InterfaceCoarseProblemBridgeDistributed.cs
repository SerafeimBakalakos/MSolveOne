using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.Solvers.DDM.Prototypes.FetiDP;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;
using MGroup.Solvers.DDM.Prototypes.PSM;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	public class InterfaceCoarseProblemBridgeDistributed : IInterfaceCoarseProblemBridge
	{
		private PsmSubdomainDofs psmDofs;
		private FetiDPSubdomainDofs fetiDPDofs;

		public BlockMatrix MatrixNcbe { get; set; }


		public Dictionary<int, Matrix> SubdomainMatricesNcb { get; } = new Dictionary<int, Matrix>();

		public void InitializeComponents(PFetiDPSolver solver)
		{
			this.psmDofs = solver.PsmDofs;
			this.fetiDPDofs = solver.FetiDPDofs;
		}

		public void LinkInterfaceCoarseProblems()
		{
			MatrixNcbe = BlockMatrix.Create(fetiDPDofs.NumSubdomainDofsCorner, psmDofs.NumSubdomainDofsBoundary);
			foreach (int s in psmDofs.NumSubdomainDofsBoundary.Keys)
			{
				// Free to boundary dofs
				int[] boundaryToFree = psmDofs.SubdomainDofsBoundaryToFree[s];
				var freeToBoundary = new Dictionary<int, int>();
				for (int i = 0; i < boundaryToFree.Length; i++)
				{
					freeToBoundary[boundaryToFree[i]] = i;
				}

				// Corner to boundary dofs
				int[] cornerToFree = fetiDPDofs.SubdomainDofsCornerToFree[s];
				var Ncb = Matrix.CreateZero(cornerToFree.Length, boundaryToFree.Length);
				for (int c = 0; c < cornerToFree.Length; c++)
				{
					int f = cornerToFree[c];
					bool exists = freeToBoundary.TryGetValue(f, out int b); // all corner dofs are also boundary.
					Debug.Assert(exists, "Found corner dof that is not boundary. This should not have happened");
					Ncb[c, b] = 1.0;
				}
				this.SubdomainMatricesNcb[s] = Ncb;
				this.MatrixNcbe.AddBlock(s, s, Ncb);
			}
		}
	}
}
