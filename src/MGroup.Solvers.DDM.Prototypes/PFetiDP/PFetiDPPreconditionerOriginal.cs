using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.Prototypes.FetiDP;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	public class PFetiDPPreconditionerOriginal : IPFetiDPPreconditioner
	{
		private FetiDPSubdomainStiffnesses fetiDPStiffnesses;
		private FetiDPCoarseProblemOriginal coarseProblem;
		private PFetiDPHomogeneousScalingOriginal pfetiDPScaling;
		private InterfaceCoarseProblemBridgeOriginal interfaceCoarseProblemBridge;
		private Matrix matrix;

		public void Calculate()
		{
			Matrix Lce = coarseProblem.MatrixLce.CopyToFullMatrix();
			Matrix Lpre = pfetiDPScaling.MatrixLpre;
			Matrix Ncb = interfaceCoarseProblemBridge.GlobalMatrixNcb;
			Matrix Kcre = fetiDPStiffnesses.Kcre.CopyToFullMatrix();
			Matrix Krce = fetiDPStiffnesses.Krce.CopyToFullMatrix();
			Matrix invKrre = fetiDPStiffnesses.InvKrre.CopyToFullMatrix();
			Matrix invScc = coarseProblem.InvScc;
			Matrix A1 = Lpre.Transpose() * invKrre * Lpre;
			Matrix A2 = Ncb.Transpose() - Lpre.Transpose() * invKrre * Krce * Lce;
			Matrix A3 = Ncb - Lce.Transpose() * Kcre * invKrre * Lpre;
			this.matrix = A1 + A2 * invScc * A3;
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
			var input = (Vector)rhsVector;
			var output = (Vector)lhsVector;
			matrix.MultiplyIntoResult(input, output);
		}
	}
}
