using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.DDM.LinearSystem
{
	public class SubdomainLinearSystem<TMatrix> : ISubdomainLinearSystem
		where TMatrix : class, IMatrix
	{
		private readonly DistributedAlgebraicModel<TMatrix> algebraicModel;
		private readonly int subdomainID;

		public SubdomainLinearSystem(DistributedAlgebraicModel<TMatrix> algebraicModel, int subdomainID)
		{
			this.subdomainID = subdomainID;
			this.algebraicModel = algebraicModel;
		}

		public ISubdomainFreeDofOrdering DofOrdering => algebraicModel.DofOrdering.SubdomainDofOrderings[subdomainID];

		public TMatrix Matrix
		{
			get => algebraicModel.LinearSystem.Matrix.LocalMatrices[subdomainID];
		}

		public Vector RhsVector 
		{
			get => algebraicModel.LinearSystem.RhsVector.LocalVectors[subdomainID];
		}

		public Vector Solution 
		{
			get => algebraicModel.LinearSystem.Solution.LocalVectors[subdomainID];
			set => algebraicModel.LinearSystem.Solution.LocalVectors[subdomainID] = value;
		}
	}
}
