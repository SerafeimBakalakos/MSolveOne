using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Matrices.Builders;
using MGroup.LinearAlgebra.Triangulation;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.LinearSystem;
using MGroup.Solvers.Logging;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Enrichment.Observers;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class ReanalysisRebuildingSolver : ISolver, IDisposable
	{
		private const string name = "ReanalysisRebuildingSolver"; // for error messages
		private const bool useSuperNodalFactorization = true; // For faster back/forward substitutions.
		private readonly double factorizationPivotTolerance;
		private readonly ReanalysisAlgebraicModel<DokMatrixAdapter> algebraicModel;

		private readonly NewCrackStepNodesObserver newStepNodes = new NewCrackStepNodesObserver();
		private readonly NewCrackTipNodesObserver newTipNodes = new NewCrackTipNodesObserver();
		private readonly PreviousCrackTipNodesObserver previousTipNodes = new PreviousCrackTipNodesObserver();

		private CholeskySuiteSparse factorization;
		private int iteration = 0;

		private ReanalysisRebuildingSolver(ReanalysisAlgebraicModel<DokMatrixAdapter> algebraicModel,
			double factorizationPivotTolerance)
		{
			this.algebraicModel = algebraicModel;
			this.factorizationPivotTolerance = factorizationPivotTolerance;
			this.LinearSystem = algebraicModel.LinearSystem;

			LinearSystem.Observers.Add(this);
			this.Logger = new SolverLogger(name);

			INodeEnricher enricher = algebraicModel.Model.GeometryModel.Enricher;
			enricher.Observers.Add(newStepNodes);
			enricher.Observers.Add(newTipNodes);
			enricher.Observers.Add(previousTipNodes);
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
			//TODO: These should not be called at all when doing reanalysis. 
			//TODO: Reanalysis should be redesigned at the analyzer and algebraic system level.
			//if (factorization != null)
			//{
			//	factorization.Dispose();
			//	factorization = null;
			//}
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
			if (iteration == 0)
			{
				FactorizeMatrix();
			}
			else
			{
				UpdateMatrixFactorization();
			}

			// Substitutions
			var watch = new Stopwatch();
			watch.Start();
			ResetSolution();
			factorization.SolveLinearSystem(LinearSystem.RhsVector.SingleVector, LinearSystem.Solution.SingleVector);
			watch.Stop();
			Logger.LogTaskDuration("Back/forward substitutions", watch.ElapsedMilliseconds);

			++iteration;
			Logger.IncrementAnalysisStep();
		}

		private void FactorizeMatrix()
		{
			var watch = new Stopwatch();
			watch.Start();

			// Matrix assembly
			SymmetricCscMatrix matrix = LinearSystem.Matrix.SingleMatrix.DokMatrix.BuildSymmetricCscMatrix(true);

			// Factorization
			factorization = CholeskySuiteSparse.Factorize(matrix, useSuperNodalFactorization);

			watch.Stop();
			Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
		}

		private void ReleaseResources()
		{
			if (factorization != null)
			{
				factorization.Dispose();
				factorization = null;
			}
		}

		private void ResetSolution()
		{
			if (LinearSystem.Solution.SingleVector == null)
			{
				int systemSize = LinearSystem.RhsVector.Length;
				LinearSystem.Solution.SingleVector = Vector.CreateZero(systemSize);
			}
			else
			{
				LinearSystem.Solution.Clear();// no need to waste computational time on this in a direct solver
			}
		}

		private void UpdateMatrixFactorization()
		{
			var watch = new Stopwatch();
			watch.Start();

			if (factorization == null)
			{
				throw new InvalidOperationException();
			}
			DokSymmetric matrix = LinearSystem.Matrix.SingleMatrix.DokMatrix;

			// Find the dofs that were added
			var colsToAdd = new SortedSet<int>();
			FindNewCrackStepDofs(colsToAdd);
			FindNewCrackTipDofs(colsToAdd);

			// Find the dofs that were removed
			var colsToRemove = new HashSet<int>(colsToAdd);
			FindPreviousCrackTipDofs(colsToRemove);

			// Delete columns corresponding to removed dofs
			foreach (int col in colsToRemove)
			{
				factorization.DeleteRow(col);
			}

			// Add columns corresponding to new dofs
			foreach (int col in colsToAdd)
			{
				colsToRemove.Remove(col);
				SparseVector newColVector = matrix.GetColumnWithoutRows(col, colsToRemove);
				factorization.AddRow(col, newColVector);
			}

			watch.Stop();
			Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
		}

		private void FindNewCrackStepDofs(SortedSet<int> dofIndices)
		{
			ActiveDofs allDofs = algebraicModel.Model.AllDofs;
			IntDofTable dofOrdering = algebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			foreach (XNode node in newStepNodes.NewCrackStepNodes)
			{
				foreach (EnrichmentItem enrichment in node.Enrichments)
				{
					if (enrichment.EnrichmentFunctions[0] is IStepEnrichment)
					{
						foreach (IDofType dof in enrichment.EnrichedDofs)
						{
							int dofID = allDofs.GetIdOfDof(dof);
							dofIndices.Add(dofOrdering[node.ID, dofID]);
						}
					}
				}
			}
		}

		private void FindNewCrackTipDofs(SortedSet<int> dofIndices)
		{
			ActiveDofs allDofs = algebraicModel.Model.AllDofs;
			IntDofTable dofOrdering = algebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			foreach (XNode node in newTipNodes.NewCrackTipNodes)
			{
				foreach (EnrichmentItem enrichment in node.Enrichments)
				{
					if (enrichment.EnrichmentFunctions[0] is ICrackTipEnrichment)
					{
						foreach (IDofType dof in enrichment.EnrichedDofs)
						{
							int dofID = allDofs.GetIdOfDof(dof);
							dofIndices.Add(dofOrdering[node.ID, dofID]);
						}
					}
				}
			}
		}

		private void FindPreviousCrackTipDofs(HashSet<int> dofIndices)
		{
			ActiveDofs allDofs = algebraicModel.Model.AllDofs;
			IntDofTable dofOrdering = algebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			foreach (XNode node in previousTipNodes.PreviousCrackTipNodes)
			{
				foreach (EnrichmentItem enrichment in node.Enrichments)
				{
					if (enrichment.EnrichmentFunctions[0] is ICrackTipEnrichment)
					{
						foreach (IDofType dof in enrichment.EnrichedDofs)
						{
							int dofID = allDofs.GetIdOfDof(dof);
							dofIndices.Add(dofOrdering[node.ID, dofID]);
						}
					}
				}
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

			public ReanalysisRebuildingSolver BuildSolver(ReanalysisAlgebraicModel<DokMatrixAdapter> model)
				=> new ReanalysisRebuildingSolver(model, FactorizationPivotTolerance);

			public ReanalysisAlgebraicModel<DokMatrixAdapter> BuildAlgebraicModel(XModel<IXCrackElement> model)
				=> new ReanalysisAlgebraicModel<DokMatrixAdapter>(model, DofOrderer, new ReanalysisWholeMatrixAssembler());
		}
	}
}
