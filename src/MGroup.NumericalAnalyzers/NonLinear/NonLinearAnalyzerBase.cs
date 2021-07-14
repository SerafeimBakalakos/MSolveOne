using System;
using System.Collections.Generic;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.MSolve.AnalysisWorkflow.Providers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using ISAAR.MSolve.Logging;
using MGroup.MSolve.AnalysisWorkflow.Logging;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.NumericalAnalyzers.NonLinear
{

	/// <summary>
	/// This class represents the base class of all nonlinear anaylsers and contains the basic information necessary for other analyzers
	/// </summary>
	public abstract class NonLinearAnalyzerBase : IChildAnalyzer
	{
		protected readonly int maxIterationsPerIncrement;
		protected readonly IModel model;
		protected readonly IAlgebraicModel algebraicModel;
		protected readonly int numIncrements;
		protected readonly int numIterationsForMatrixRebuild;
		protected readonly INonLinearProvider provider;
		protected readonly double residualTolerance;
		protected readonly ISolver solver;
		protected readonly INonLinearModelUpdater modelUpdater;
		protected IGlobalVector rhs;
		protected IGlobalVector u;
		protected IGlobalVector du;
		protected IGlobalVector uPlusdu;
		protected double globalRhsNormInitial;
		protected INonLinearParentAnalyzer parentAnalyzer = null;

		internal NonLinearAnalyzerBase(IModel model, IAlgebraicModel algebraicModel, ISolver solver, INonLinearProvider provider,
			INonLinearModelUpdater modelUpdater,
			int numIncrements, int maxIterationsPerIncrement, int numIterationsForMatrixRebuild, double residualTolerance)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.solver = solver;
			this.provider = provider;
			this.modelUpdater = modelUpdater;
			this.numIncrements = numIncrements;
			this.maxIterationsPerIncrement = maxIterationsPerIncrement;
			this.numIterationsForMatrixRebuild = numIterationsForMatrixRebuild;
			this.residualTolerance = residualTolerance;
		}

		public LinearAnalyzerLogFactory LogFactory { get; set; }

		public IAnalysisWorkflowLog[] Logs { get; set; } = new IAnalysisWorkflowLog[0];

		public TotalDisplacementsPerIterationLog TotalDisplacementsPerIterationLog { get; set; }

		public IncrementalDisplacementsLog IncrementalDisplacementsLog { get; set; }

		public TotalLoadsDisplacementsPerIncrementLog IncrementalLog { get; set; }

		public IParentAnalyzer ParentAnalyzer
		{
			get => parentAnalyzer;
			set => parentAnalyzer = (INonLinearParentAnalyzer)value;
		}

		public IGlobalVector Responses { get; set; }

		/// <summary>
		/// Builds the tangent stiffness matrix of the system.
		/// </summary>
		public void BuildMatrices()
		{
			if (parentAnalyzer == null)
			{
				throw new InvalidOperationException(
				"This Newton-Raphson nonlinear analyzer has no parent.");
			}

			parentAnalyzer.BuildMatrices();
		}

		/// <summary>
		/// Initializes internal vector before the first analysis.
		/// </summary>
		public void Initialize(bool isFirstAnalysis)
		{
			InitializeInternalVectors();
		}

		protected IGlobalVector CalculateInternalRhs(int currentIncrement, int iteration)
		{
			if (currentIncrement == 0 && iteration == 0)
			{
				du.Clear();
				uPlusdu.Clear();
				du.AddIntoThis(solver.LinearSystem.Solution);
				uPlusdu.AddIntoThis(solver.LinearSystem.Solution);
				du.SubtractIntoThis(u);
			}
			else
			{
				du.AddIntoThis(solver.LinearSystem.Solution);
				uPlusdu.Clear();
				uPlusdu.AddIntoThis(u);
				uPlusdu.AddIntoThis(du);
			}
			IGlobalVector internalRhs = modelUpdater.GetRhsFromSolution(uPlusdu);
			provider.ProcessInternalRhs(uPlusdu, internalRhs);

			if (parentAnalyzer != null)
			{
				IGlobalVector otherRhsComponents = parentAnalyzer.GetOtherRhsComponents(uPlusdu);
				internalRhs.AddIntoThis(otherRhsComponents);
			}

			return internalRhs;
		}

		protected double UpdateResidualForcesAndNorm(int currentIncrement, IGlobalVector internalRhs)
		{
			solver.LinearSystem.RhsVector.Clear();
			for (int j = 0; j <= currentIncrement; j++)
			{
				solver.LinearSystem.RhsVector.AddIntoThis(rhs);
			}
			solver.LinearSystem.RhsVector.SubtractIntoThis(internalRhs);
			return provider.CalculateRhsNorm(solver.LinearSystem.RhsVector);
		}

		protected void ClearIncrementalSolutionVector()
		{
			du.Clear();
		}

		protected virtual void InitializeInternalVectors()
		{
			rhs = solver.LinearSystem.RhsVector.Copy();
			rhs.ScaleIntoThis(1 / (double)numIncrements);
			u = algebraicModel.CreateZeroVector();
			du = algebraicModel.CreateZeroVector();
			uPlusdu = algebraicModel.CreateZeroVector();
			globalRhsNormInitial = provider.CalculateRhsNorm(solver.LinearSystem.RhsVector);
		}

		protected void InitializeLogs()
		{
			if (LogFactory != null)
			{
				Logs = LogFactory.CreateLogs();
			}
			if (IncrementalLog != null)
			{
				IncrementalLog.Initialize();
			}
		}

		protected void SaveMaterialStateAndUpdateSolution()
		{
			modelUpdater.UpdateState();
			u.AddIntoThis(du);
		}

		protected void StoreLogResults(DateTime start, DateTime end)
		{
			foreach (var l in Logs)
			{
				l.StoreResults(start, end, u);
			}
		}

		protected void UpdateInternalVectors()
		{
			rhs = solver.LinearSystem.RhsVector.Copy();
			rhs.ScaleIntoThis(1 / (double)numIncrements);
			globalRhsNormInitial = provider.CalculateRhsNorm(solver.LinearSystem.RhsVector);
		}

		protected void UpdateRhs(int step)
		{
			solver.LinearSystem.RhsVector.CopyFrom(rhs);
		}

		/// <summary>
		/// This class solves system and calculates the displacements vector.
		/// </summary>
		public abstract void Solve();
	}
}
