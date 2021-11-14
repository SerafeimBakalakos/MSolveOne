using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Triangulation;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.LinearSystem;
using MGroup.Solvers.Logging;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class ReanalysisRebuildingSolver : ISolver, IDisposable
	{
		protected const string name = "ReanalysisRebuildingSolver"; // for error messages
		private const bool useSuperNodalFactorization = true; // For faster back/forward substitutions.
		private readonly double factorizationPivotTolerance;

		protected readonly GlobalAlgebraicModel<DokMatrixAdapter> algebraicModel;
		private bool mustFactorize = true;
		private CholeskySuiteSparse factorization;

		private ReanalysisRebuildingSolver(GlobalAlgebraicModel<DokMatrixAdapter> algebraicModel,
			double factorizationPivotTolerance)
		{
			this.factorizationPivotTolerance = factorizationPivotTolerance;
			this.LinearSystem = algebraicModel.LinearSystem;
			LinearSystem.Observers.Add(this);

			this.Logger = new SolverLogger(name);
		}

		IGlobalLinearSystem ISolver.LinearSystem => LinearSystem;

		public GlobalLinearSystem<DokMatrixAdapter> LinearSystem { get; set; }

		public ISolverLogger Logger { get; }

		public string Name => name;

		~ReanalysisRebuildingSolver()
		{
			ReleaseResources();
		}

		public void Dispose()
		{
			ReleaseResources();
			GC.SuppressFinalize(this);
		}

		public void HandleMatrixWillBeSet()
		{
			mustFactorize = true;
			if (factorization != null)
			{
				factorization.Dispose();
				factorization = null;
			}
			//TODO: make sure the native memory allocated has been cleared. We need all the available memory we can get.
		}

		public void PreventFromOverwrittingSystemMatrices()
		{
			// The factorization is done over different memory.
		}

		/// <summary>
		/// Solves the linear system with back-forward substitution. If the matrix has been modified, it will be refactorized.
		/// </summary>
		public void Solve()
		{
			var watch = new Stopwatch();
			SymmetricCscMatrix matrix = LinearSystem.Matrix.SingleMatrix.DokMatrix.BuildSymmetricCscMatrix(true);
			int systemSize = matrix.NumRows;
			if (LinearSystem.Solution.SingleVector == null)
			{
				LinearSystem.Solution.SingleVector = Vector.CreateZero(systemSize);
			}
			else LinearSystem.Solution.Clear();// no need to waste computational time on this in a direct solver

			// Factorization
			if (mustFactorize)
			{
				watch.Start();
				factorization = CholeskySuiteSparse.Factorize(matrix, useSuperNodalFactorization);
				watch.Stop();
				Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
				watch.Reset();
				mustFactorize = false;
			}

			// Substitutions
			watch.Start();
			factorization.SolveLinearSystem(LinearSystem.RhsVector.SingleVector, LinearSystem.Solution.SingleVector);
			watch.Stop();
			Logger.LogTaskDuration("Back/forward substitutions", watch.ElapsedMilliseconds);
			Logger.IncrementAnalysisStep();
		}

		private void ReleaseResources()
		{
			if (factorization != null)
			{
				factorization.Dispose();
				factorization = null;
			}
		}

		public class Factory
		{
			public Factory(ReanalysisDofOrderer dofOrderer) 
			{
				this.DofOrderer = dofOrderer;
			}

			public IDofOrderer DofOrderer { get; set; }

			public double FactorizationPivotTolerance { get; set; } = 1E-15;

			public ReanalysisRebuildingSolver BuildSolver(GlobalAlgebraicModel<DokMatrixAdapter> model)
				=> new ReanalysisRebuildingSolver(model, FactorizationPivotTolerance);

			public GlobalAlgebraicModel<DokMatrixAdapter> BuildAlgebraicModel(IModel model)
				=> new GlobalAlgebraicModel<DokMatrixAdapter>(model, DofOrderer, new ReanalysisWholeMatrixAssembler());
		}
	}
}
