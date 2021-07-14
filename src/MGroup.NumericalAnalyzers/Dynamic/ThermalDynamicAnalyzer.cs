using System;
using System.Collections.Generic;
using System.Diagnostics;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.MSolve.AnalysisWorkflow.Providers;
using MGroup.NumericalAnalyzers.Logging;
//using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.MSolve.AnalysisWorkflow.Logging;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.NumericalAnalyzers.Dynamic
{
	/// <summary>
	/// This class makes the appropriate arrangements for the solution of thermal dynamic equations
	/// according to the Central Diffences method.
	/// Authors: Yannis Kalogeris
	/// </summary>
	public class ThermalDynamicAnalyzer : INonLinearParentAnalyzer
	{
		private readonly double beta, timeStep, totalTime;
		private readonly IModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ISolver solver;
		private readonly IImplicitIntegrationProvider provider;
		private IGlobalVector rhs;
		private IGlobalVector rhsPrevious;
		private IGlobalVector temperature;
		private IGlobalVector capacityTimesTemperature;
		private IGlobalVector conductivityTimesTemperature;

		/// <summary>
		/// Creates an instance that uses a specific problem type and an appropriate child analyzer for the construction of the system of equations arising from the actual physical problem
		/// </summary>
		/// <param name="model">Instance of the model to be solved</param>
		/// <param name="solver">Instance of the solver that will handle the solution of the system of equations</param>
		/// <param name="provider">Instance of the problem type to be solver</param>
		/// <param name="childAnalyzer">Instance of the child analyzer that will handle the solution of the system of equations</param>
		/// <param name="beta">Instance of parameter "beta" of the method that will be initialized</param>
		/// <param name="timeStep">Instance of the time step of the method that will be initialized</param>
		/// <param name="totalTime">Instance of the total time of the method that will be initialized</param>
		public ThermalDynamicAnalyzer(IModel model, IAlgebraicModel algebraicModel, ISolver solver, IImplicitIntegrationProvider provider,
			IChildAnalyzer childAnalyzer, double beta, double timeStep, double totalTime)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.solver = solver;
			this.provider = provider;
			this.ChildAnalyzer = childAnalyzer;
			this.beta = beta;
			this.timeStep = timeStep;
			this.totalTime = totalTime;
			this.ChildAnalyzer.ParentAnalyzer = this;
		}

		public IAnalysisWorkflowLog[] Logs => null;

		public IChildAnalyzer ChildAnalyzer { get; }

		/// <summary>
		/// Makes the proper solver-specific initializations before the solution of the linear system of equations. This method MUST be called before the actual solution of the aforementioned system
		/// </summary>
		public void BuildMatrices()
		{
			var coeffs = new ImplicitIntegrationCoefficients
			{
				Mass = 1 / timeStep,
				Stiffness = beta,
			};
			//TODO: REmove accordingly
			provider.LinearCombinationOfMatricesIntoStiffness(coeffs);
			//foreach (ILinearSystem linearSystem in solver.LinearSystems.Values)
			//{
			//	linearSystem.Matrix = provider.LinearCombinationOfMatricesIntoStiffness(coeffs, linearSystem.Subdomain);
			//}
		}

		///// <summary>
		///// Calculates inertia forces.
		///// </summary>
		//public IVector GetOtherRhsComponents(ILinearSystem linearSystem, IVector currentSolution)
		//{
		//	return provider.MassMatrixVectorProduct(linearSystem.Subdomain, currentSolution);
		//}

		/// <summary>
		/// Calculates inertia forces.
		/// </summary>
		public IGlobalVector GetOtherRhsComponents(IGlobalVector currentSolution)
		{
			return provider.MassMatrixVectorProduct(currentSolution);
		}

		/// <summary>
		/// Initializes the models, the solvers, child analyzers, builds the matrices, assigns loads and initializes right-hand-side vectors.
		/// </summary>
		public void Initialize(bool isFirstAnalysis = true)
		{
			if (isFirstAnalysis)
			{
				model.ConnectDataStructures();
				algebraicModel.OrderDofs();
			}

			BuildMatrices();

			provider.AssignRhs();

			InitializeInternalVectors();

			InitializeRhs();

			if (ChildAnalyzer == null)
			{
				throw new InvalidOperationException("Newmark analyzer must contain an embedded analyzer.");
			}

			ChildAnalyzer.Initialize(isFirstAnalysis);
		}

		/// <summary>
		/// Solves the linear system of equations by calling the corresponding method of the specific solver attached during construction of the current instance
		/// </summary>
		public void Solve()
		{
			int numTimeSteps = (int)(totalTime / timeStep);
			for (int t = 0; t < numTimeSteps; ++t)
			{
				Debug.WriteLine("Newmark step: {0}", t);

				IGlobalVector rhsVector = provider.GetRhsFromHistoryLoad(t);
				solver.LinearSystem.RhsVector = rhsVector; //TODOGoat: Perhaps the provider should set the rhs vector, like it does for the matrix. Either way the provider does this as a side effect

				InitializeRhs();
				CalculateRhsImplicit();

				DateTime start = DateTime.Now;
				ChildAnalyzer.Solve();
				DateTime end = DateTime.Now;

				UpdateTemperature(t);
				UpdateResultStorages(start, end);
			}
		}

		/// <summary>
		/// Calculates the right-hand-side of the implicit dyanmic method. This will be used for the solution of the linear dynamic system.
		/// </summary>
		private void CalculateRhsImplicit()
		{
			capacityTimesTemperature = provider.MassMatrixVectorProduct(temperature);
			conductivityTimesTemperature = provider.DampingMatrixVectorProduct(temperature);

			IGlobalVector rhsResult = rhsPrevious.LinearCombination(1 - beta, rhs, beta);
			rhsResult.AxpyIntoThis(capacityTimesTemperature, 1 / timeStep);
			rhsResult.AxpyIntoThis(conductivityTimesTemperature, -(1 - beta));

			rhsPrevious = rhs;

			solver.LinearSystem.RhsVector = rhsResult;
		}

		private void InitializeInternalVectors()
		{
			capacityTimesTemperature = algebraicModel.CreateZeroVector();
			conductivityTimesTemperature = algebraicModel.CreateZeroVector();
			rhs = algebraicModel.CreateZeroVector();
			rhsPrevious = algebraicModel.CreateZeroVector();

			if (solver.LinearSystem.Solution != null)
			{
				temperature = solver.LinearSystem.Solution.Copy();
			}
			else
			{
				temperature = algebraicModel.CreateZeroVector();
			}
		}

		private void InitializeRhs()
		{
			ImplicitIntegrationCoefficients coeffs = new ImplicitIntegrationCoefficients
			{
				Mass = 0,
				Stiffness = 0,
			};
			provider.ProcessRhs(coeffs, solver.LinearSystem.RhsVector);
			rhs.CopyFrom(solver.LinearSystem.RhsVector);
		}

		private void UpdateResultStorages(DateTime start, DateTime end)
		{
			//foreach (ILinearSystem linearSystem in solver.LinearSystems.Values)
			//{
			//	int id = linearSystem.Subdomain.ID;
			//	if (ResultStorages.ContainsKey(id))
			//	{
			//		if (ResultStorages[id] != null)
			//		{
			//			foreach (var l in ChildAnalyzer.Logs[id])
			//			{
			//				ResultStorages[id].StoreResults(start, end, l);
			//			}
			//		}
			//	}
			//}
		}

		private void UpdateTemperature(int timeStep)
		{
			temperature.CopyFrom(solver.LinearSystem.Solution);
		}
	}
}
