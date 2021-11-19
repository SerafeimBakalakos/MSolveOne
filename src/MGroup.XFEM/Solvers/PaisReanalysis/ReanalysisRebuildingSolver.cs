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
		private readonly double factorizationPivotTolerance;
		private readonly ReanalysisAlgebraicModel<DokMatrixAdapter> algebraicModel;

		private readonly NewCrackStepNodesObserver newStepNodes;
		private readonly CrackStepNodesWithModifiedLevelSetObserver stepNodesWithModifiedLevelSet;
		private readonly NewCrackTipNodesObserver newTipNodes;
		private readonly PreviousCrackTipNodesObserver previousTipNodes;
		private readonly NodesWithModifiedEnrichmentsObserver nodesWithModifiedEnrichments;
		private readonly ElementsWithModifiedNodesObserver elementsWithModifiedNodes;
		private readonly NodesNearModifiedNodesObserver nodesNearModifiedNodes;

		private CholeskySuiteSparse factorization;
		private int iteration = 0;
		private HashSet<int> inactiveDofsPrevious = new HashSet<int>();

		private ReanalysisRebuildingSolver(ReanalysisAlgebraicModel<DokMatrixAdapter> algebraicModel,
			double factorizationPivotTolerance)
		{
			this.algebraicModel = algebraicModel;
			this.factorizationPivotTolerance = factorizationPivotTolerance;
			this.LinearSystem = algebraicModel.LinearSystem;

			LinearSystem.Observers.Add(this);
			this.Logger = new SolverLogger(name);

			// Enrichment observers
			newStepNodes = new NewCrackStepNodesObserver();
			stepNodesWithModifiedLevelSet = new CrackStepNodesWithModifiedLevelSetObserver(algebraicModel.Model);
			newTipNodes = new NewCrackTipNodesObserver();
			previousTipNodes = new PreviousCrackTipNodesObserver();
			nodesWithModifiedEnrichments = new NodesWithModifiedEnrichmentsObserver(
				newStepNodes, stepNodesWithModifiedLevelSet, newTipNodes, previousTipNodes);
			elementsWithModifiedNodes = new ElementsWithModifiedNodesObserver(nodesWithModifiedEnrichments);
			nodesNearModifiedNodes = new NodesNearModifiedNodesObserver(nodesWithModifiedEnrichments, elementsWithModifiedNodes);

			var compositeObserver = new CompositeEnrichmentObserver();
			compositeObserver.AddObservers(newStepNodes, stepNodesWithModifiedLevelSet, newTipNodes, previousTipNodes, 
				nodesWithModifiedEnrichments, elementsWithModifiedNodes, nodesNearModifiedNodes);
			INodeEnricher enricher = algebraicModel.Model.GeometryModel.Enricher;
			enricher.Observers.Add(compositeObserver);
		}

		IGlobalLinearSystem ISolver.LinearSystem => LinearSystem;

		public GlobalLinearSystem<DokMatrixAdapter> LinearSystem { get; set; }

		public ISolverLogger Logger { get; }

		public string Name => name;

		#region debug delete
		public IEnrichedNodeSelector NodeSelector { get; set; }
		#endregion

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

			UpdateInactiveDofs();

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

			#region debug uncomment
			//// Find enriched dofs that were added
			//var colsToAdd = new SortedSet<int>();
			//FindCrackStepDofs(newStepNodes.NewCrackStepNodes, colsToAdd);
			//FindCrackTipDofs(newTipNodes.NewCrackTipNodes, colsToAdd);

			//// Find enriched dofs that were removed
			//var colsToRemove = new HashSet<int>(colsToAdd);
			//FindCrackTipDofs(previousTipNodes.PreviousCrackTipNodes, colsToRemove);

			//// Find enriched dofs that have modified stiffness and possibly enrichment values
			//var colsToModify = new HashSet<int>();
			//FindCrackStepDofs(stepNodesWithModifiedLevelSet.StepNodesWithModifiedLevelSets, colsToModify);
			//FindCrackStepDofs(nodesNearModifiedNodes.NearModifiedNodes, colsToModify);
			//colsToRemove.UnionWith(colsToModify);
			//colsToAdd.UnionWith(colsToModify);
			#endregion

			#region debug delete
			//IntDofTable dofOrdering = algebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			//ActiveDofs allDofs = algebraicModel.Model.AllDofs;
			//var colsToAdd = new SortedSet<int>();
			//var colsToRemove = new HashSet<int>();

			//// All dofs
			//for (int i = 0; i < matrix.NumColumns; ++i)
			//{
			//	colsToAdd.Add(i);
			//}

			// All enriched dofs.
			////The next causes errors. Why? Perhaps I should also remove the std stiffnesses that interact with the modified 
			////enriched dofs. Perhaps this was not a problem in previous examples, because the ordering helped prevent modified 
			////Kse entries showing up in the upper triangle.
			//foreach ((int nodeID, int dofID, int col) in dofOrdering)
			//{
			//	IDofType dofType = allDofs.GetDofWithId(dofID);
			//	if (dofType is EnrichedDof)
			//	{
			//		colsToAdd.Add(col);
			//	}
			//}

			//// All dofs of possibly enriched nodes
			//var modifiedNodes = new HashSet<XNode>();
			//foreach (XNode node in algebraicModel.Model.Nodes.Values)
			//{
			//	if (NodeSelector.CanNodeBeEnriched(node))
			//	{
			//		modifiedNodes.Add(node);
			//	}
			//}
			//FindAllDofs(modifiedNodes, colsToAdd);

			//// All dofs of nodes near modified dofs
			//HashSet<XNode> modifiedNodes = FindAllModifiedNodes();
			//FindAllDofs(modifiedNodes, colsToAdd);


			//FindCrackStepDofs(newStepNodes.NewCrackStepNodes, colsToAdd);
			//FindCrackTipDofs(newTipNodes.NewCrackTipNodes, colsToAdd);
			//colsToRemove = new HashSet<int>(colsToAdd);
			//FindCrackTipDofs(previousTipNodes.PreviousCrackTipNodes, colsToRemove);

			//(ISet<int> colsToAdd, ISet<int> colsToRemove) = FindAllModifiedDofs();
			(ISet<int> colsToAdd, ISet<int> colsToRemove) = ClassifyColumns();


			Console.WriteLine($"colsToAdd={colsToAdd.Count}, colsToRemove={colsToRemove.Count}");
			#endregion

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
				//SparseVector newColVector = matrix.GetColumn(col);
				factorization.AddRow(col, newColVector);
			}

			UpdateInactiveDofs();
			watch.Stop();
			Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
		}

		private (ISet<int> colsToAdd, ISet<int> colsToRemove) ClassifyColumns()
		{
			var colsToAdd = new HashSet<int>();
			var colsToRemove = new HashSet<int>();
			var colsToModify = new HashSet<int>();

			// Dofs with modified stiffness (diagonal entries)
			FindCrackStepDofs(newStepNodes.NewCrackStepNodes, colsToAdd);
			FindCrackTipDofs(newTipNodes.NewCrackTipNodes, colsToAdd);
			FindCrackTipDofs(previousTipNodes.PreviousCrackTipNodes, colsToRemove);
			FindCrackStepDofs(stepNodesWithModifiedLevelSet.StepNodesWithModifiedLevelSets, colsToModify);
			colsToAdd.UnionWith(colsToModify);
			colsToRemove.UnionWith(colsToModify);
			colsToModify.UnionWith(colsToAdd);
			colsToModify.UnionWith(colsToRemove);

			// Nodes with dofs with modified stiffness or near them
			HashSet<XNode> modifiedNodes = FindNodesNearModifiedDofs();

			// Dofs with unmodified diagonal entries, but modified super-diagonal entries
			IntDofTable dofOrdering = algebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			DokSymmetric matrix = LinearSystem.Matrix.SingleMatrix.DokMatrix;
			foreach (XNode node in modifiedNodes)
			{
				List<int> nearbyDofs = FindNearbyDofs(node);
				if (node.ID == 1482)
				{
					Console.WriteLine();
				}
				foreach (var pair in dofOrdering.GetDataOfRow(node.ID))
				{
					int dofID = pair.Key;
					int col = pair.Value;
					if (colsToModify.Contains(col))
					{
						// We only care about dofs with unmodified stiffness now
						continue;
					}

					Dictionary<int, double> wholeColumn = matrix.RawColumns[col];
					if (wholeColumn.Count == 1 || wholeColumn[col] == 1.0)
					{
						// Inactive dof
						continue;
					}

					if (MustUpdateNearModifiedDof(node.ID, dofID, col, nearbyDofs, colsToModify))
					{
						colsToAdd.Add(col);
						colsToRemove.Add(col);
						//colsToModify.UnionWith(colsToRemove); //ERROR: Leave this alone or all these dofs will be updated redundantly.
					}
				}
			}


			//IntDofTable dofOrdering = algebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			//DokSymmetric matrix = LinearSystem.Matrix.SingleMatrix.DokMatrix;
			//foreach (XNode node in modifiedNodes)
			//{
			//	foreach (int col in dofOrdering.GetValuesOfRow(node.ID))
			//	{
			//		if (colsToModify.Contains(col))
			//		{
			//			continue;
			//		}

			//		colsToRemove.Add(col);
			//		colsToAdd.Add(col);
			//	}
			//}

			// Optimization to avoid redundant adds
			colsToAdd.RemoveWhere(i => (matrix.RawColumns[i].Count == 1) && (matrix.RawColumns[i][i] == 1.0));
			colsToRemove.RemoveWhere(i => inactiveDofsPrevious.Contains(i));

			return (new SortedSet<int>(colsToAdd), new SortedSet<int>(colsToRemove));
		}

		private bool MustUpdateNearModifiedDof(int nodeID, int dofID, int col, List<int> nearbyDofs, HashSet<int> colsToModify)
		{
			return true;
			//ActiveDofs allDofs = algebraicModel.Model.AllDofs;
			//if (allDofs.GetDofWithId(dofID) is EnrichedDof)
			//{
			//	return false;
			//}
			//else
			//{
			//	return true;
			//}

			int j = col;
			foreach (int i in nearbyDofs)
			{
				if (colsToModify.Contains(i))
				{
					// At this point we have an unmodified dof with a neighboring modified one
					if (i < j)
					{
						// The coupling stiffness entry Aij belongs to column j and has been modified. 
						// Thus the whole column needs to be modified.
						return true;
					}
				}
			}
			return false;
		}

		private void UpdateInactiveDofs()
		{
			inactiveDofsPrevious.Clear();
			int numColumns = LinearSystem.Matrix.SingleMatrix.DokMatrix.NumColumns;
			var matrixColumns = LinearSystem.Matrix.SingleMatrix.DokMatrix.RawColumns;
			for (int i = 0; i < numColumns; ++i)
			{
				Dictionary<int, double> column = matrixColumns[i];
				if ((column.Count == 1) && (column[i] == 1.0))
				{
					inactiveDofsPrevious.Add(i);
				}
			}
		}

		private (ISet<int> colsToAdd, ISet<int> colsToRemove) FindAllModifiedDofs()
		{
			var modifiedNodes = new HashSet<XNode>();
			modifiedNodes.UnionWith(newStepNodes.NewCrackStepNodes);
			modifiedNodes.UnionWith(newTipNodes.NewCrackTipNodes);
			modifiedNodes.UnionWith(previousTipNodes.PreviousCrackTipNodes);
			modifiedNodes.UnionWith(stepNodesWithModifiedLevelSet.StepNodesWithModifiedLevelSets);
			var modifiedElements = new HashSet<IXFiniteElement>();
			foreach (XNode node in modifiedNodes)
			{
				modifiedElements.UnionWith(node.ElementsDictionary.Values);
			}
			foreach (IXFiniteElement element in modifiedElements)
			{
				modifiedNodes.UnionWith(element.Nodes);
			}

			var colsToAdd = new SortedSet<int>();
			IntDofTable dofOrdering = algebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			foreach (XNode node in modifiedNodes)
			{
				IReadOnlyDictionary<int, int> dofsOfNode = dofOrdering.GetDataOfRow(node.ID);
				foreach (var dofIDIndexPair in dofsOfNode)
				{
					colsToAdd.Add(dofIDIndexPair.Value);
				}
			}
			var colsToRemove = new HashSet<int>(colsToAdd);

			// Optimization to avoid redundant adds and removes
			DokSymmetric matrix = LinearSystem.Matrix.SingleMatrix.DokMatrix;
			colsToAdd.RemoveWhere(i => (matrix.RawColumns[i].Count == 1) && (matrix.RawColumns[i][i] == 1.0));
			colsToRemove.RemoveWhere(i => inactiveDofsPrevious.Contains(i));

			return (new SortedSet<int>(colsToAdd), new SortedSet<int>(colsToRemove));
		}

		private List<int> FindNearbyDofs(XNode node)
		{
			var nearbyNodes = new HashSet<XNode>();
			foreach (IXFiniteElement element in node.ElementsDictionary.Values)
			{
				nearbyNodes.UnionWith(element.Nodes);
			}

			IntDofTable dofOrdering = algebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			var nearbyDofs = new List<int>();
			foreach (XNode otherNode in nearbyNodes)
			{
				nearbyDofs.AddRange(dofOrdering.GetValuesOfRow(otherNode.ID));
			}
			return nearbyDofs;
		}

		private HashSet<XNode> FindNodesNearModifiedDofs()
		{
			var modifiedNodes = new HashSet<XNode>();
			modifiedNodes.UnionWith(newStepNodes.NewCrackStepNodes);
			modifiedNodes.UnionWith(newTipNodes.NewCrackTipNodes);
			modifiedNodes.UnionWith(previousTipNodes.PreviousCrackTipNodes);
			modifiedNodes.UnionWith(stepNodesWithModifiedLevelSet.StepNodesWithModifiedLevelSets);

			var modifiedElements = new HashSet<IXFiniteElement>();
			foreach (XNode node in modifiedNodes)
			{
				modifiedElements.UnionWith(node.ElementsDictionary.Values);
			}
			foreach (IXFiniteElement element in modifiedElements)
			{
				modifiedNodes.UnionWith(element.Nodes);
			}
			return modifiedNodes;
		}

		private void FindAllDofs(IEnumerable<XNode> inNodes, ISet<int> dofIndices)
		{
			IntDofTable dofOrdering = algebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			foreach (XNode node in inNodes)
			{
				IReadOnlyDictionary<int, int> dofsOfNode = dofOrdering.GetDataOfRow(node.ID);
				foreach (var dofIDIndexPair in dofsOfNode)
				{
					dofIndices.Add(dofIDIndexPair.Value);
				}
			}
		}

		private void FindCrackStepDofs(IEnumerable<XNode> inNodes, ISet<int> dofIndices)
		{
			ActiveDofs allDofs = algebraicModel.Model.AllDofs;
			IntDofTable dofOrdering = algebraicModel.SubdomainFreeDofOrdering.FreeDofs;
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

		private void FindCrackTipDofs(IEnumerable<XNode> inNodes, ISet<int> dofIndices)
		{
			ActiveDofs allDofs = algebraicModel.Model.AllDofs;
			IntDofTable dofOrdering = algebraicModel.SubdomainFreeDofOrdering.FreeDofs;
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
