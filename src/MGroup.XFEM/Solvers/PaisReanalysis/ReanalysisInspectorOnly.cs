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
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class ReanalysisInspectorOnly : ReanalysisRebuildingSolver
	{
		private const string name = "ReanalysisInspectorOnly"; // for error messages
		private readonly IReanalysisExtraDofsStrategy extraDofsStrategy;

		private int iteration = 0;

		private ReanalysisInspectorOnly(ReanalysisAlgebraicModel<DokMatrixAdapter> algebraicModel, 
			IReanalysisExtraDofsStrategy extraDofsStrategy, double factorizationPivotTolerance)
			: base(algebraicModel, extraDofsStrategy, factorizationPivotTolerance)
		{
			this.extraDofsStrategy = extraDofsStrategy;
			extraDofsStrategy.Solver = this;
		}

		~ReanalysisInspectorOnly()
		{
			ReleaseResources();
		}

		/// <summary>
		/// Solves the linear system with back-forward substitution. If the matrix has been modified, it will be refactorized.
		/// </summary>
		public override void Solve()
		{
			(int numStdDofs, int numEnrDofs) = CountStdEnrDofs();
			int numModifiedDofs, numAddedDofs, numRemovedDofs, numInactiveDofs;
			if (iteration == 0)
			{
				UpdateInactiveDofs();
				numInactiveDofs = PreviouslyInactiveDofs.Count;
				numAddedDofs = numStdDofs + numEnrDofs - numInactiveDofs;
				numRemovedDofs = 0;
				numModifiedDofs = numAddedDofs;
			}
			else
			{
				(ISet<int> dofsToAdd, ISet<int> dofsToRemove, _) = FindModifiedColumns();
				ISet<int> modifiedDofs = new HashSet<int>(dofsToAdd);
				modifiedDofs.UnionWith(dofsToRemove);
				UpdateInactiveDofs();

				numAddedDofs = dofsToAdd.Count;
				numRemovedDofs = dofsToRemove.Count;
				numModifiedDofs = modifiedDofs.Count;
				numInactiveDofs = PreviouslyInactiveDofs.Count;
			}

			LinearSystem.Solution.Clear();

			Logger.LogNumDofs("Standard dofs", numStdDofs);
			Logger.LogNumDofs("Active enriched dofs", numEnrDofs - numInactiveDofs);
			Logger.LogNumDofs("Inactive enriched dofs", numInactiveDofs);
			Logger.LogNumDofs("Modified dofs", numModifiedDofs);
			Logger.LogNumDofs("Added dofs", numAddedDofs);
			Logger.LogNumDofs("Deleted dofs", numRemovedDofs);
			Logger.IncrementAnalysisStep();

			++iteration;
		}

		private (int numStdDofs, int numEnrDofs) CountStdEnrDofs()
		{
			ActiveDofs allDofs = this.AlgebraicModel.Model.AllDofs;
			IntDofTable freeDofs = this.AlgebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			int numStdDofs = 0;
			int numEnrDofs = 0;
			foreach ((_, int dofID, _) in freeDofs)
			{
				IDofType dof = allDofs.GetDofWithId(dofID);
				if (dof is EnrichedDof)
				{
					++numEnrDofs;
				}
				else
				{
					++numStdDofs;
				}
			}
			return (numStdDofs, numEnrDofs);
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
				=> new ReanalysisInspectorOnly(model, ExtraDofsStrategy, FactorizationPivotTolerance);

			public ReanalysisAlgebraicModel<DokMatrixAdapter> BuildAlgebraicModel(XModel<IXCrackElement> model)
				=> new ReanalysisAlgebraicModel<DokMatrixAdapter>(model, DofOrderer, new ReanalysisWholeMatrixAssembler());
		}
	}
}
