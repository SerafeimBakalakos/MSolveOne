using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Iterative.Termination;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;
using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.Solvers.DDM.Prototypes.FetiDP
{
	public class FetiDPCoarseProblemDistributedJacobi : FetiDPCoarseProblemDistributed
	{

		public FetiDPCoarseProblemDistributedJacobi(IModel model, FetiDPSubdomainDofs dofs, 
			FetiDPSubdomainStiffnesses stiffnesses, double pcgResidualTolerance)
			: base(model, dofs, stiffnesses, pcgResidualTolerance)

		{
		}

		protected override IPreconditioner CreatePreconditioner()
		{
			BlockVector diagScc = stiffnesses.Scce.ExtractDiagonal();
			diagScc = MatrixMce * diagScc;
			BlockVector inverseDiagScc = diagScc.DoToAllEntries(x => 1 / x);
			return new JacobiPreconditioner(inverseDiagScc);
		}

		private class JacobiPreconditioner : IPreconditioner
		{
			public BlockVector inverseDiagScc;

			public JacobiPreconditioner(BlockVector inverseDiagScc)
			{
				this.inverseDiagScc = inverseDiagScc;
			}

			public void SolveLinearSystem(IVectorView rhsVector, IVector lhsVector)
			{
				lhsVector.CopyFrom(inverseDiagScc.MultiplyEntryWise(rhsVector));
			}
		}
	}
}
