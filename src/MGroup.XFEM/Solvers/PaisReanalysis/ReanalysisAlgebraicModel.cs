using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Distributed.LinearAlgebraExtensions;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution.AlgebraicModel;

using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.LinearSystem;
using MGroup.Solvers.Results;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class ReanalysisAlgebraicModel<TMatrix> : GlobalAlgebraicModel<TMatrix>
		where TMatrix : class, IMatrix
	{
		private readonly ISubdomain subdomain;
		private readonly IModel model;
		private readonly IDofOrderer dofOrderer;
		private readonly ISubdomainMatrixAssembler<TMatrix> subdomainMatrixAssembler;
		private readonly SubdomainVectorAssembler subdomainVectorAssembler;

		private bool orderingExists = false;

		public ReanalysisAlgebraicModel(IModel model, IDofOrderer dofOrderer,
			ISubdomainMatrixAssembler<TMatrix> subdomainMatrixAssembler) : base(model, dofOrderer, subdomainMatrixAssembler)
		{
			this.model = model;
			this.dofOrderer = dofOrderer;
			this.subdomainMatrixAssembler = subdomainMatrixAssembler;
			subdomain = model.EnumerateSubdomains().First();
		}

		public override void OrderDofs()
		{
			if (!orderingExists)
			{
				base.OrderDofs();
			}
			orderingExists = true;
		}

		public override void ReorderDofs()
		{
			this.OrderDofs();
		}
	}
}
