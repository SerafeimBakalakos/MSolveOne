using System;
using System.Collections.Generic;
using DotNumerics.ODE.Radau5;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.MSolve.AnalysisWorkflow.Logging;
using MGroup.MSolve.AnalysisWorkflow.Providers;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;

namespace MGroup.NumericalAnalyzers
{
	/// <summary>
	/// This class solves the linear system.
	/// </summary>
	public class LinearAnalyzer : IChildAnalyzer
	{
		private readonly IModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly IAnalyzerProvider provider;
		private readonly ISolver solver;


		/// <summary>
		/// This class defines the linear anaylzer.
		/// </summary>
		/// <param name="model">Instance of the model that will be solved</param>
		/// <param name="solver">Instance of the solver that will solve the linear system of equations</param>
		/// <param name="provider">Instance of the problem type to be solved</param> 
		public LinearAnalyzer(IModel model, IAlgebraicModel algebraicModel, ISolver solver, IAnalyzerProvider provider)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.solver = solver;
			this.provider = provider;
		}

		public ILogFactory LogFactory { get; set; }

		public IAnalysisWorkflowLog[] Logs { get; set; } = new IAnalysisWorkflowLog[0];

		public IParentAnalyzer ParentAnalyzer { get; set; }

		public IGlobalVector Responses { get; set; } 

		public void BuildMatrices()
		{
			if (ParentAnalyzer == null)
			{
				throw new InvalidOperationException("This linear analyzer has no parent.");
			}

			ParentAnalyzer.BuildMatrices();
		}

		public void Initialize(bool isFirstAnalysis)
		{
			InitializeLogs();
		}

		public void Solve()
		{
			var start = DateTime.Now;
			AddEquivalentNodalLoadsToRHS();
			solver.Solve();
			Responses = solver.LinearSystem.Solution.Copy();
			var end = DateTime.Now;
			StoreLogResults(start, end);
		}

		private void AddEquivalentNodalLoadsToRHS()
		{
			//TODO: equivalentNodalLoads will often be 0. Perhaps instead of AddToGlobalVector, we should have AxpyToGlobalVector
			IGlobalVector equivalentNodalLoads = algebraicModel.CreateZeroVector();
			algebraicModel.AddToGlobalVector(model.EnumerateDirichletBoundaryConditions, equivalentNodalLoads);
			solver.LinearSystem.RhsVector.SubtractIntoThis(equivalentNodalLoads); 
		}

		private void InitializeLogs()
		{
			if (LogFactory != null)
			{
				Logs = LogFactory.CreateLogs();
			}
		}

		private void StoreLogResults(DateTime start, DateTime end)
		{
			foreach (var l in Logs)
			{
				l.StoreResults(start, end, solver.LinearSystem.Solution);
			}
		}
	}
}
