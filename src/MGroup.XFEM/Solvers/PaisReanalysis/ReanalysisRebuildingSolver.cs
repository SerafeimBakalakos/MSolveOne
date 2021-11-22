using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
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
		private readonly IReanalysisExtraDofsStrategy extraDofsStrategy;
		private readonly double factorizationPivotTolerance;

		private CholeskySuiteSparse factorization;
		private int iteration = 0;

		private ReanalysisRebuildingSolver(ReanalysisAlgebraicModel<DokMatrixAdapter> algebraicModel, 
			IReanalysisExtraDofsStrategy extraDofsStrategy, double factorizationPivotTolerance)
		{
			this.AlgebraicModel = algebraicModel;
			this.extraDofsStrategy = extraDofsStrategy;
			extraDofsStrategy.Solver = this;
			this.factorizationPivotTolerance = factorizationPivotTolerance;
			this.LinearSystem = algebraicModel.LinearSystem;

			LinearSystem.Observers.Add(this);
			this.Logger = new SolverLogger(name);

			// Enrichment observers
			NewStepNodes = new NewCrackStepNodesObserver();
			StepNodesWithModifiedLevelSet = new CrackStepNodesWithModifiedLevelSetObserver(algebraicModel.Model);
			NewTipNodes = new NewCrackTipNodesObserver();
			PreviousTipNodes = new PreviousCrackTipNodesObserver();
			NodesWithModifiedEnrichments = new NodesWithModifiedEnrichmentsObserver(
				NewStepNodes, StepNodesWithModifiedLevelSet, NewTipNodes, PreviousTipNodes);
			ElementsWithModifiedNodes = new ElementsWithModifiedNodesObserver(NodesWithModifiedEnrichments);
			NodesNearModifiedNodes = new NodesNearModifiedNodesObserver(NodesWithModifiedEnrichments, ElementsWithModifiedNodes);

			var compositeObserver = new CompositeEnrichmentObserver();
			compositeObserver.AddObservers(NewStepNodes, StepNodesWithModifiedLevelSet, NewTipNodes, PreviousTipNodes, 
				NodesWithModifiedEnrichments, ElementsWithModifiedNodes, NodesNearModifiedNodes);
			INodeEnricher enricher = algebraicModel.Model.GeometryModel.Enricher;
			enricher.Observers.Add(compositeObserver);
		}

		public ReanalysisAlgebraicModel<DokMatrixAdapter> AlgebraicModel { get; }

		public NewCrackStepNodesObserver NewStepNodes { get; }

		public CrackStepNodesWithModifiedLevelSetObserver StepNodesWithModifiedLevelSet { get; }

		public NewCrackTipNodesObserver NewTipNodes { get; }

		public PreviousCrackTipNodesObserver PreviousTipNodes { get; }

		public NodesWithModifiedEnrichmentsObserver NodesWithModifiedEnrichments { get; }

		public ElementsWithModifiedNodesObserver ElementsWithModifiedNodes { get; }

		public NodesNearModifiedNodesObserver NodesNearModifiedNodes { get; }

		IGlobalLinearSystem ISolver.LinearSystem => LinearSystem;

		public GlobalLinearSystem<DokMatrixAdapter> LinearSystem { get; set; }

		public ISolverLogger Logger { get; }

		public string Name => name;

		public IEnrichedNodeSelector NodeSelector { get; set; }

		public HashSet<int> PreviouslyInactiveDofs { get; } = new HashSet<int>();

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

			// Matrix assembly
			SymmetricCscMatrix matrix = LinearSystem.Matrix.SingleMatrix.DokMatrix.BuildSymmetricCscMatrix(true);

			// Factorization
			watch.Start();
			factorization = CholeskySuiteSparse.Factorize(matrix, useSuperNodalFactorization);

			UpdateInactiveDofs();

			watch.Stop();
			Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
		}

		public void FindCrackStepDofs(IEnumerable<XNode> inNodes, ISet<int> dofIndices)
		{
			ActiveDofs allDofs = AlgebraicModel.Model.AllDofs;
			IntDofTable dofOrdering = AlgebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			foreach (XNode node in inNodes)
			{
				IReadOnlyDictionary<int, int> dofsOfNode = dofOrdering.GetDataOfRow(node.ID);
				foreach (var dofIDIndexPair in dofsOfNode)
				{
					IDofType dofType = allDofs.GetDofWithId(dofIDIndexPair.Key);
					if (dofType is EnrichedDof enrichedDof)
					{
						if (enrichedDof.Enrichment is IStepEnrichment)
						{
							dofIndices.Add(dofIDIndexPair.Value);
						}
					}
				}
			}
		}

		public void FindCrackTipDofs(IEnumerable<XNode> inNodes, ISet<int> dofIndices)
		{
			ActiveDofs allDofs = AlgebraicModel.Model.AllDofs;
			IntDofTable dofOrdering = AlgebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			foreach (XNode node in inNodes)
			{
				IReadOnlyDictionary<int, int> dofsOfNode = dofOrdering.GetDataOfRow(node.ID);
				foreach (var dofIDIndexPair in dofsOfNode)
				{
					IDofType dofType = allDofs.GetDofWithId(dofIDIndexPair.Key);
					if (dofType is EnrichedDof enrichedDof)
					{
						if (enrichedDof.Enrichment is ICrackTipEnrichment)
						{
							dofIndices.Add(dofIDIndexPair.Value);
						}
					}
				}
			}
		}

		private (ISet<int> colsToAdd, ISet<int> colsToRemove) FindModifiedColumns()
		{
			var colsToAdd = new HashSet<int>();
			var colsToRemove = new HashSet<int>();

			// Dofs with modified stiffness (diagonal entries)
			FindCrackStepDofs(NewStepNodes.NewCrackStepNodes, colsToAdd);
			FindCrackTipDofs(NewTipNodes.NewCrackTipNodes, colsToAdd);
			FindCrackTipDofs(PreviousTipNodes.PreviousCrackTipNodes, colsToRemove);
			FindCrackStepDofs(StepNodesWithModifiedLevelSet.StepNodesWithModifiedLevelSets, colsToAdd);
			colsToRemove.UnionWith(colsToAdd);

			HashSet<int> extraModifiedCols = extraDofsStrategy.FindExtraModifiedCols(colsToAdd, colsToRemove);
			colsToAdd.UnionWith(extraModifiedCols);
			colsToRemove.UnionWith(extraModifiedCols);

			// Optimization to avoid redundant adds
			DokSymmetric matrix = LinearSystem.Matrix.SingleMatrix.DokMatrix;
			colsToAdd.RemoveWhere(i => (matrix.RawColumns[i].Count == 1) && (matrix.RawColumns[i][i] == 1.0));
			colsToRemove.RemoveWhere(i => PreviouslyInactiveDofs.Contains(i) && (!colsToAdd.Contains(i)));

			return (new SortedSet<int>(colsToAdd), new SortedSet<int>(colsToRemove));
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

		private void UpdateInactiveDofs()
		{
			PreviouslyInactiveDofs.Clear();
			int numColumns = LinearSystem.Matrix.SingleMatrix.DokMatrix.NumColumns;
			var matrixColumns = LinearSystem.Matrix.SingleMatrix.DokMatrix.RawColumns;
			for (int i = 0; i < numColumns; ++i)
			{
				Dictionary<int, double> column = matrixColumns[i];
				if ((column.Count == 1) && (column[i] == 1.0))
				{
					PreviouslyInactiveDofs.Add(i);
				}
			}
		}

		private void UpdateMatrixFactorization()
		{
			long updateDuration = 0;
			long managedDuration = 0;
			var watch = new Stopwatch();
			watch.Start();

			if (factorization == null)
			{
				throw new InvalidOperationException();
			}

			(ISet<int> colsToAdd, ISet<int> colsToRemove) = FindModifiedColumns();
			watch.Stop();
			managedDuration += watch.ElapsedMilliseconds;

			watch.Restart();
			// Delete columns corresponding to removed dofs
			foreach (int col in colsToRemove)
			{
				factorization.DeleteRow(col);
			}
			watch.Stop();
			updateDuration += watch.ElapsedMilliseconds;

			// Add columns corresponding to new dofs
			DokSymmetric matrix = LinearSystem.Matrix.SingleMatrix.DokMatrix;

			foreach (int col in colsToAdd)
			{
				watch.Restart();
				colsToRemove.Remove(col);
				SparseVector newColVector = matrix.GetColumnWithoutRows(col, colsToRemove);
				watch.Stop();
				managedDuration += watch.ElapsedMilliseconds;

				watch.Restart();
				factorization.AddRow(col, newColVector);
				watch.Stop();
				updateDuration += watch.ElapsedMilliseconds;
			}

			watch.Restart();
			UpdateInactiveDofs();
			watch.Stop();
			managedDuration += watch.ElapsedMilliseconds;

			Logger.LogTaskDuration("Matrix factorization", updateDuration);
			Logger.LogTaskDuration("Matrix factorization managed code", managedDuration);
			//Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
		}

		public class Factory
		{
			public Factory(ReanalysisDofOrderer dofOrderer) 
			{
				this.DofOrderer = dofOrderer;
			}

			public IDofOrderer DofOrderer { get; set; }

			public IReanalysisExtraDofsStrategy ExtraDofsStrategy { get; set; } = new AllDofsNearModifiedDofsStrategy();

			public double FactorizationPivotTolerance { get; set; } = 1E-15;

			public ReanalysisRebuildingSolver BuildSolver(ReanalysisAlgebraicModel<DokMatrixAdapter> model)
				=> new ReanalysisRebuildingSolver(model, ExtraDofsStrategy, FactorizationPivotTolerance);

			public ReanalysisAlgebraicModel<DokMatrixAdapter> BuildAlgebraicModel(XModel<IXCrackElement> model)
				=> new ReanalysisAlgebraicModel<DokMatrixAdapter>(model, DofOrderer, new ReanalysisWholeMatrixAssembler());
		}
	}
}
