using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Distributed.IterativeMethods.PCG;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Matrices;
using MGroup.Solvers.DDM.FetiDP.Vectors;
using MGroup.Solvers.DDM.LinearSystem;

namespace MGroup.Solvers.DDM.FetiDP.InterfaceProblem
{
	/// <summary>
	/// At each iteration, it checks if ||K * u - f|| / ||f|| &lt; tol, where all vectors and matrices correspond to the free 
	/// dofs of the whole model. Since this is very inefficient, it is recommended to use this class only for testing, 
	/// comparison or benchmarking purposes.
	/// </summary>
	public class ObjectiveConvergenceCriterion<TMatrix> : IPcgResidualConvergence
		where TMatrix: class, IMatrix
	{
		private readonly DistributedAlgebraicModel<TMatrix> algebraicModel;
		private readonly FetiDPSolutionRecovery solutionRecovery;

		private double normF0;

		public ObjectiveConvergenceCriterion(DistributedAlgebraicModel<TMatrix> algebraicModel,
			FetiDPSolutionRecovery solutionRecovery)
		{
			this.algebraicModel = algebraicModel;
			this.solutionRecovery = solutionRecovery;
		}

		public double EstimateResidualNormRatio(PcgAlgorithmBase pcg)
		{
			// Find solution at free dofs
			var lambda = (DistributedOverlappingVector)(pcg.Solution);
			var Uf = new DistributedOverlappingVector(algebraicModel.FreeDofIndexer);
			solutionRecovery.CalcPrimalSolution(lambda, Uf);

			IGlobalMatrix Kff = algebraicModel.LinearSystem.Matrix;
			IGlobalVector Ff = algebraicModel.LinearSystem.RhsVector;
			IGlobalVector residual = Ff.CreateZero();
			Kff.MultiplyVector(Uf, residual);
			residual.LinearCombinationIntoThis(-1.0, Ff, +1.0);

			return residual.Norm2() / normF0;
		}

		public void Initialize(PcgAlgorithmBase pcg)
		{
			normF0 = algebraicModel.LinearSystem.RhsVector.Norm2();
		}
	}
}
