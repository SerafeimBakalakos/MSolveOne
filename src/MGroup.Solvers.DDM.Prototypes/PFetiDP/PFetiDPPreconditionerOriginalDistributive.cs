using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.Prototypes.FetiDP;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	/// <summary>
	/// Like <see cref="PFetiDPPreconditionerOriginal"/>, but perform matrix-vector and vector-vector operations instead of 
	/// operations between matrices.
	/// </summary>
	public class PFetiDPPreconditionerOriginalDistributive : IPFetiDPPreconditioner
	{
		private FetiDPSubdomainStiffnesses fetiDPStiffnesses;
		private FetiDPCoarseProblemOriginal coarseProblem;
		private PFetiDPHomogeneousScalingOriginal pfetiDPScaling;
		private InterfaceCoarseProblemBridgeOriginal interfaceCoarseProblemBridge;

		private Matrix Lce;
		private Matrix Lpre;
		private Matrix Ncb;
		private Matrix Kcre;
		private Matrix Krce;
		private Matrix invKrre;
		private Matrix invScc;

		public void Calculate()
		{
			this.Lce = coarseProblem.MatrixLce.CopyToFullMatrix();
			this.Lpre = pfetiDPScaling.MatrixLpre;
			this.Ncb = interfaceCoarseProblemBridge.GlobalMatrixNcb;
			this.Kcre = fetiDPStiffnesses.Kcre.CopyToFullMatrix();
			this.Krce = fetiDPStiffnesses.Krce.CopyToFullMatrix();
			this.invKrre = fetiDPStiffnesses.InvKrre.CopyToFullMatrix();
			this.invScc = coarseProblem.InvScc;
		}

		public void InitializeComponents(PFetiDPSolver solver)
		{
			this.fetiDPStiffnesses = solver.FetiDPStiffnesses;
			this.coarseProblem = (FetiDPCoarseProblemOriginal)(solver.CoarseProblem);
			this.pfetiDPScaling = (PFetiDPHomogeneousScalingOriginal)solver.Scaling;
			this.interfaceCoarseProblemBridge = (InterfaceCoarseProblemBridgeOriginal)solver.InterfaceCoarseProblemBridge;
		}

		public void SolveLinearSystem(IVectorView rhsVector, IVector lhsVector)
		{
			var y = (Vector)rhsVector;
			var x = (Vector)lhsVector;

			Vector xb1 = Lpre.Transpose() * (invKrre * (Lpre * y));
			Vector yc31 = Ncb * y;
			Vector yc32 = -1 * (Lce.Transpose() * (Kcre * (invKrre * (Lpre * y))));
			Vector yc = yc31 + yc32;
			Vector xc = invScc * yc;
			Vector xb21 = Ncb.Transpose() * xc;
			Vector xb22 = -1 * (Lpre.Transpose() * (invKrre * (Krce * (Lce * xc))));
			x.CopyFrom(xb1);
			x.AddIntoThis(xb21 + xb22);
		}
	}
}
