using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.Prototypes.FetiDP;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;
using MGroup.Solvers.DDM.Prototypes.PSM;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
	public class PFetiDPPreconditionerDistributedAll : IPFetiDPPreconditioner
	{
		private FetiDPSubdomainStiffnesses fetiDPStiffnesses;
		private FetiDPCoarseProblemDistributed coarseProblem;
		private PsmInterfaceProblemDistributed psmInterfaceProblem;
		private PFetiDPSubdomainDofs pfetiDPDofs;
		private PFetiDPHomogeneousScalingDistributed pfetiDPScaling;
		private InterfaceCoarseProblemBridgeDistributed interfaceCoarseProblemBridge;

		private IVectorMultipliable A1;
		private IVectorMultipliable A21;
		private IVectorMultipliable A22;
		private IVectorMultipliable A31;
		private IVectorMultipliable A32;

		public void Calculate()
		{
			BlockMatrix Mbe = psmInterfaceProblem.MatrixMbe;
			BlockMatrix Wbe = pfetiDPScaling.MatrixWbe;
			BlockMatrix Nrbe = pfetiDPDofs.MatrixNrbe;
			BlockMatrix Ncbe = interfaceCoarseProblemBridge.MatrixNcbe;
			BlockMatrix Kcre = fetiDPStiffnesses.Kcre;
			BlockMatrix Krce = fetiDPStiffnesses.Krce;
			BlockMatrix invKrre = fetiDPStiffnesses.InvKrre;

			int[][] multiplicitiesBoundary = Mbe.ColMultiplicities;
			int[][] multiplicitiesCorner = coarseProblem.MatrixMce.ColMultiplicities;
			Wbe.RowMultiplicities = Wbe.ColMultiplicities = multiplicitiesBoundary;
			Nrbe.RowMultiplicities = Nrbe.ColMultiplicities = multiplicitiesBoundary;
			Ncbe.RowMultiplicities = multiplicitiesCorner;
			Ncbe.ColMultiplicities = multiplicitiesBoundary;
			Kcre.RowMultiplicities = multiplicitiesCorner;
			Kcre.ColMultiplicities = multiplicitiesBoundary;
			Krce.RowMultiplicities = multiplicitiesBoundary;
			Krce.ColMultiplicities = multiplicitiesCorner;
			invKrre.RowMultiplicities = invKrre.ColMultiplicities = multiplicitiesBoundary;

			this.A1 = new MatrixProduct(Mbe, Wbe, Nrbe.Transpose(), invKrre, Nrbe, Wbe);
			this.A21 = Ncbe.Transpose();
			this.A22 = new MatrixProduct(Mbe, Wbe, Nrbe.Transpose(), invKrre, Krce);
			this.A31 = new MatrixProduct(Ncbe, Wbe);
			this.A32 = new MatrixProduct(Kcre, invKrre, Nrbe, Wbe);
		}

		public void InitializeComponents(PFetiDPSolver solver)
		{
			this.fetiDPStiffnesses = solver.FetiDPStiffnesses;
			this.coarseProblem = (FetiDPCoarseProblemDistributed)(solver.CoarseProblem);
			this.psmInterfaceProblem = (PsmInterfaceProblemDistributed)solver.InterfaceProblem;
			this.pfetiDPDofs = solver.PFetiDPDofs;
			this.pfetiDPScaling = (PFetiDPHomogeneousScalingDistributed)solver.Scaling;
			this.interfaceCoarseProblemBridge =
				(InterfaceCoarseProblemBridgeDistributed)solver.InterfaceCoarseProblemBridge;
		}

		public void SolveLinearSystem(IVectorView rhsVector, IVector lhsVector)
		{
			IVector xb1 = A1.Multiply(rhsVector);
			IVector yc31 = A31.Multiply(rhsVector);
			IVector yc32 = A32.Multiply(rhsVector).Scale(-1);
			IVector yce = yc31.Add(yc32);
			IVector xce = coarseProblem.Solve(yce);
			IVector xb21 = A21.Multiply(xce);
			IVector xb22 = A22.Multiply(xce).Scale(-1);
			lhsVector.CopyFrom(xb1);
			lhsVector.AddIntoThis(xb21);
			lhsVector.AddIntoThis(xb22);
		}
	}
}
