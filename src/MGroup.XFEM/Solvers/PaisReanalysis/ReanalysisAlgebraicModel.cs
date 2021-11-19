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
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class ReanalysisAlgebraicModel<TMatrix> : GlobalAlgebraicModel<TMatrix>
		where TMatrix : class, IMatrix
	{
		private bool orderingExists = false;

		public ReanalysisAlgebraicModel(XModel<IXCrackElement> model, IDofOrderer dofOrderer,
			ISubdomainMatrixAssembler<TMatrix> subdomainMatrixAssembler) : base(model, dofOrderer, subdomainMatrixAssembler)
		{
			this.Model = model;
		}

		public XModel<IXCrackElement> Model { get; }

		public override void OrderDofs()
		{
			if (!orderingExists)
			{
				base.OrderDofs();
				orderingExists = true;
			}
			else
			{
				LinearSystem.RhsVector.Clear();
				LinearSystem.Solution.Clear();
			}
		}

		public override void ReorderDofs()
		{
			this.OrderDofs();
		}
	}
}
