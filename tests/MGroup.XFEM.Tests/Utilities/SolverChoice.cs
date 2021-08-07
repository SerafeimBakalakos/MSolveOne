using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Direct;
using MGroup.Solvers.Iterative;

namespace MGroup.XFEM.Tests.Utilities
{
	public enum SolverChoice
	{
		Dense, Skyline, SuiteSparse, PCG
	}

	public static class SolverChoiceExtensions
	{
		public static (IAlgebraicModel algebraicModel, ISolver solver) Create(this SolverChoice choice, IModel model)
		{
			if (choice == SolverChoice.Dense)
			{
				var factory = new DenseMatrixSolver.Factory();
				GlobalAlgebraicModel<Matrix> algebraicModel = factory.BuildAlgebraicModel(model);
				var solver = factory.BuildSolver(algebraicModel);
				return (algebraicModel, solver);
			}
			else if (choice == SolverChoice.Skyline)
			{
				var factory = new SkylineSolver.Factory();
				GlobalAlgebraicModel<SkylineMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
				var solver = factory.BuildSolver(algebraicModel);
				return (algebraicModel, solver);
			}
			else if (choice == SolverChoice.SuiteSparse)
			{
				var factory = new SuiteSparseSolver.Factory();
				GlobalAlgebraicModel<SymmetricCscMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
				var solver = factory.BuildSolver(algebraicModel);
				return (algebraicModel, solver);
			}
			else if (choice == SolverChoice.PCG)
			{
				var factory = new PcgSolver.Factory();
				GlobalAlgebraicModel<CsrMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
				var solver = factory.BuildSolver(algebraicModel);
				return (algebraicModel, solver);
			}
			else
			{
				throw new NotImplementedException();
			}
		}
	}

}
