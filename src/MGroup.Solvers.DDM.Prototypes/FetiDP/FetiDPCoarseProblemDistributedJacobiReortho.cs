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
	public class FetiDPCoarseProblemDistributedJacobiReortho : FetiDPCoarseProblemDistributedJacobi
	{

		public FetiDPCoarseProblemDistributedJacobiReortho(IModel model, FetiDPSubdomainDofs dofs, 
			FetiDPSubdomainStiffnesses stiffnesses, double pcgResidualTolerance)
			: base(model, dofs, stiffnesses, pcgResidualTolerance)

		{
			var iterativeSolverBuilder = new LinearAlgebraExtensions.ReorthogonalizedPcg.Builder();
			iterativeSolverBuilder.MaxIterationsProvider = new PercentageMaxIterationsProvider(1.0);
			iterativeSolverBuilder.ResidualTolerance = 0.02 * pcgResidualTolerance; // HERE: Let user define this.
			IterativeSolver = iterativeSolverBuilder.Build();
		}
	}
}
