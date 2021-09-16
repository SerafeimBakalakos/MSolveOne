using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Distributed.IterativeMethods.PCG;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PSM.Vectors;

namespace MGroup.Solvers.DDM.PSM.InterfaceProblem
{
	/// <summary>
	/// At each iteration, it checks if ||K * u - f|| / ||f|| &lt; tol, where all vectors and matrices correspond to the free 
	/// dofs of the whole model. Since this is very inefficient, it is recommended to use this class only for testing, 
	/// comparison or benchmarking purposes.
	/// </summary>
	public class ObjectiveConvergenceCriterion<TMatrix> : IPcgResidualConvergence
		where TMatrix: class, IMatrix
	{
		private readonly IComputeEnvironment environment;
		private readonly DistributedAlgebraicModel<TMatrix> algebraicModel;
		private readonly Func<int, PsmSubdomainVectors> getSubdomainVectors;

		private double normF0;

		public ObjectiveConvergenceCriterion(IComputeEnvironment environment, DistributedAlgebraicModel<TMatrix> algebraicModel, 
			Func<int, PsmSubdomainVectors> getSubdomainVectors)
		{
			this.environment = environment;
			this.algebraicModel = algebraicModel;
			this.getSubdomainVectors = getSubdomainVectors;
		}

		public double EstimateResidualNormRatio(PcgAlgorithmBase pcg)
		{
			// Find displacements at free dofs
			var Ub = (DistributedOverlappingVector)(pcg.Solution);
			var Uf = new DistributedOverlappingVector(algebraicModel.FreeDofIndexer);
			environment.DoPerNode(subdomainID =>
			{
				Vector ubs = Ub.LocalVectors[subdomainID];
				Vector ufs = getSubdomainVectors(subdomainID).CalcSubdomainFreeSolution(ubs);
				Uf.LocalVectors[subdomainID] = ufs;
			});

			IGlobalMatrix Kff = algebraicModel.LinearSystem.Matrix;
			IGlobalVector Ff = algebraicModel.LinearSystem.RhsVector;
			IGlobalVector residual = Ff.CreateZero();
			Kff.MultiplyVector(Uf, residual);
			residual.LinearCombinationIntoThis(-1.0, Ff, +1.0);

			return residual.Norm2();
		}

		public void Initialize(PcgAlgorithmBase pcg)
		{
			normF0 = algebraicModel.LinearSystem.RhsVector.Norm2();
		}
	}
}
