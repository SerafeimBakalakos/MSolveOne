using System;
using System.Collections.Generic;
using System.Diagnostics;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.MSolve.AnalysisWorkflow.Providers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.MSolve.AnalysisWorkflow.Logging;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.NumericalAnalyzers.Dynamic
{
	public class NewmarkDynamicAnalyzer : INonLinearParentAnalyzer
	{
		/// <summary>
		/// This class makes the appropriate arrangements for the solution of linear dynamic equations
		/// according to implicit Newmark method
		/// Authors: George Stavroulakis, George Soimiris
		/// </summary>
		private readonly double beta;

		/// <summary>
		/// This class makes the appropriate arrangements for the solution of linear dynamic equations
		/// according to implicit Newmark method
		/// Authors: George Stavroulakis, George Soimiris
		/// </summary>
		private readonly double gamma;

		/// <summary>
		/// This class makes the appropriate arrangements for the solution of linear dynamic equations
		/// according to implicit Newmark method
		/// Authors: George Stavroulakis, George Soimiris
		/// </summary>
		private readonly double timeStep;

		/// <summary>
		/// This class makes the appropriate arrangements for the solution of linear dynamic equations
		/// according to implicit Newmark method
		/// Authors: George Stavroulakis, George Soimiris
		/// </summary>
		private readonly double totalTime;
		private readonly double a0;
		private readonly double a1;
		private readonly double a2;
		private readonly double a3;
		private readonly double a4;
		private readonly double a5;
		private readonly double a6;
		private readonly double a7;
		private readonly IModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ISolver solver;
		private readonly IImplicitIntegrationProvider provider;
		private IGlobalVector rhs;
		private IGlobalVector uu;
		private IGlobalVector uum;
		private IGlobalVector uc;
		private IGlobalVector ucc;
		private IGlobalVector u;
		private IGlobalVector v;
		private IGlobalVector v1;
		private IGlobalVector v2;

		/// <summary>
		/// Creates an instance that uses a specific problem type and an appropriate child analyzer for the construction of the system of equations arising from the actual physical problem
		/// </summary>
		/// <param name="model">Instance of the model to be solved</param>
		/// <param name="solver">Instance of the solver that will handle the solution of the system of equations</param>
		/// <param name="provider">Instance of the problem type to be solver</param>
		/// <param name="childAnalyzer">Instance of the child analyzer that will handle the solution of the system of equations</param>
		/// <param name="timeStep">Instance of the time step of the method that will be initialized</param>
		/// <param name="totalTime">Instance of the total time of the method that will be initialized</param>
		/// <param name="alpha">Instance of parameter "alpha" of the method that will be initialized</param>
		/// <param name="delta">Instance of parameter "delta" of the method that will be initialized</param>
		private NewmarkDynamicAnalyzer(IModel model, IAlgebraicModel algebraicModel, ISolver solver, IImplicitIntegrationProvider provider,
			IChildAnalyzer childAnalyzer, double timeStep, double totalTime, double alpha, double delta)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.solver = solver;
			this.provider = provider;
			this.ChildAnalyzer = childAnalyzer;
			this.beta = alpha;
			this.gamma = delta;
			this.timeStep = timeStep;
			this.totalTime = totalTime;
			this.ChildAnalyzer.ParentAnalyzer = this;

			/// <summary>
			/// Initialize coefficients. It would make sense for them to be initialized in a different function, if they could
			/// change during the analysis
			/// </summary>
			a0 = 1 / (alpha * timeStep * timeStep);
			a1 = delta / (alpha * timeStep);
			a2 = 1 / (alpha * timeStep);
			a3 = (1 / (2 * alpha)) - 1;
			a4 = (delta / alpha) - 1;
			a5 = timeStep * 0.5 * ((delta / alpha) - 2);
			a6 = timeStep * (1 - delta);
			a7 = delta * timeStep;
		}

		public IAnalysisWorkflowLog[] Logs => null;

		public ImplicitIntegrationAnalyzerLog ResultStorage { get; set; }

		public IChildAnalyzer ChildAnalyzer { get; }

		/// <summary>
		/// Makes the proper solver-specific initializations before the solution of the linear system of equations. This method MUST be called before the actual solution of the aforementioned system
		/// </summary>
		public void BuildMatrices()
		{
			var coeffs = new ImplicitIntegrationCoefficients
			{
				Mass = a0,
				Damping = a1,
				Stiffness = 1,
			};
			provider.LinearCombinationOfMatricesIntoStiffness(coeffs);
		}

		/// <summary>
		/// Calculates inertia forces and damping forces.
		/// </summary>
		public IGlobalVector GetOtherRhsComponents(IGlobalVector currentSolution)
		{
			IGlobalVector result = provider.MassMatrixVectorProduct(currentSolution);
			IGlobalVector temp = provider.DampingMatrixVectorProduct(currentSolution);
			result.LinearCombinationIntoThis(a0, temp, a1);
			return result;
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
			for (int i = 0; i < numTimeSteps; ++i)
			{
				Debug.WriteLine("Newmark step: {0}", i);

				IGlobalVector rhsVector = provider.GetRhsFromHistoryLoad(i);
				solver.LinearSystem.RhsVector = rhsVector; //TODOGoat: Perhaps the provider should set the rhs vector, like it does for the matrix. Either way the provider does this as a side effect

				InitializeRhs();
				CalculateRhsImplicit();

				DateTime start = DateTime.Now;
				ChildAnalyzer.Solve();
				DateTime end = DateTime.Now;

				UpdateVelocityAndAcceleration(i);
				UpdateResultStorages(start, end);
			}
		}

		/// <summary>
		/// Calculates the right-hand-side of the implicit dyanmic method. This will be used for the solution of the linear dynamic system.
		/// </summary>
		private void CalculateRhsImplicit()
		{
			uu = v.LinearCombination(a0, v1, a2);
			uu.AxpyIntoThis(v2, a3);

			uc = v.LinearCombination(a1, v1, a4);
			uc.AxpyIntoThis(v2, a5);

			uum = provider.MassMatrixVectorProduct(uu);
			ucc = provider.DampingMatrixVectorProduct(uc);

			IGlobalVector rhsResult = uum.Add(ucc);
			bool addRhs = true;
			if (addRhs)
			{
				rhsResult.AddIntoThis(rhs);
			}
			solver.LinearSystem.RhsVector = rhsResult;
		}

		private void InitializeInternalVectors()
		{
			uu = algebraicModel.CreateZeroVector();
			uum = algebraicModel.CreateZeroVector();
			uc = algebraicModel.CreateZeroVector();
			ucc = algebraicModel.CreateZeroVector();
			u = algebraicModel.CreateZeroVector();
			v1 = algebraicModel.CreateZeroVector();
			v2 = algebraicModel.CreateZeroVector();
			rhs = algebraicModel.CreateZeroVector();

			if (solver.LinearSystem.Solution != null)
			{
				v = solver.LinearSystem.Solution.Copy();
			}
			else
			{
				v = algebraicModel.CreateZeroVector();
			}
		}

		private void InitializeRhs()
		{
			ImplicitIntegrationCoefficients coeffs = new ImplicitIntegrationCoefficients
			{
				Mass = a0,
				Damping = a1,
				Stiffness = 1,
			};
			provider.ProcessRhs(coeffs, solver.LinearSystem.RhsVector);
			rhs.CopyFrom(solver.LinearSystem.RhsVector);
		}

		private void UpdateResultStorages(DateTime start, DateTime end)
		{
			if (ResultStorage != null)
			{
				foreach (var l in ChildAnalyzer.Logs)
				{
					ResultStorage.StoreResults(start, end, l);
				}
			}
		}

		private void UpdateVelocityAndAcceleration(int timeStep)
		{
			IGlobalVector externalVelocities = provider.GetVelocitiesOfTimeStep(timeStep);
			IGlobalVector externalAccelerations = provider.GetAccelerationsOfTimeStep(timeStep);

			u.CopyFrom(v);
			v.CopyFrom(solver.LinearSystem.Solution);

			IGlobalVector vv = v2.Add(externalAccelerations);

			v2 = v.Subtract(u);
			v2.LinearCombinationIntoThis(a0, v1, -a2);
			v2.AxpyIntoThis(vv, -a3);

			v1.AddIntoThis(externalVelocities);
			v1.AxpyIntoThis(vv, a6);
			v1.AxpyIntoThis(v2, a7);
		}

		public class Builder
		{
			private readonly double timeStep;
			private readonly double totalTime;
			private readonly IChildAnalyzer childAnalyzer;
			private readonly IModel model;
			private readonly IAlgebraicModel algebraicModel;
			private readonly ISolver solver;
			private readonly IImplicitIntegrationProvider provider;
			private double beta = 0.25;
			private double gamma = 0.5;

			public Builder(IModel model, IAlgebraicModel algebraicModel, ISolver solver, IImplicitIntegrationProvider provider,
				IChildAnalyzer childAnalyzer, double timeStep, double totalTime)
			{
				this.model = model;
				this.algebraicModel = algebraicModel;
				this.solver = solver;
				this.provider = provider;
				this.childAnalyzer = childAnalyzer;

				this.timeStep = timeStep;
				this.totalTime = totalTime;
			}

			/// <summary>
			///
			/// </summary>
			/// <param name="beta">
			/// Used in the intepolation between the accelerations of the previous and current time step, in order to obtain the
			/// current displacements. Also called alpha by Bathe.
			/// </param>
			/// <param name="gamma">
			/// Used in the intepolation between the accelerations of the previous and current time step, in order to obtain the
			/// current velocities. Also called delta by Bathe.
			/// </param>
			/// <param name="allowConditionallyStable">
			/// If set to true, the user must make sure that the time step chosen is lower than the critical step size
			/// corresponding to these particular <paramref name="beta"/>, <paramref name="gamma"/> parameters.
			/// </param>
			public void SetNewmarkParameters(double beta, double gamma, bool allowConditionallyStable = false)
			{
				if (!allowConditionallyStable)
				{
					if (gamma < 0.5)
					{
						throw new ArgumentException(
						"Newmark delta has to be bigger than 0.5 to ensure unconditional stability.");
					}

					if (beta < 0.25)
					{
						throw new ArgumentException(
						"Newmark alpha has to be bigger than 0.25 to ensure unconditional stability.");
					}
				}
				if (gamma < 0.5)
				{
					throw new ArgumentException("Newmark delta has to be bigger than 0.5.");
				}

				double aLimit = 0.25 * Math.Pow(0.5 + gamma, 2);
				if (beta < aLimit)
				{
					throw new ArgumentException($"Newmark alpha has to be bigger than {aLimit}.");
				}

				this.gamma = gamma;
				this.beta = beta;
			}

			/// <summary>
			/// Central diffences: gamma = 1/2, beta = 0. Newmark results in central diffences, a conditionally stable explicit
			/// method. To ensure stability, the time step must be &lt;= the critical step size = 2 / w,  where w is the maximum
			/// natural radian frequency. It would be more efficient to use an explicit dynamic analyzer.
			/// </summary>
			public void SetNewmarkParametersForCentralDifferences()
			{
				gamma = 0.5;
				beta = 0.0;
			}

			/// <summary>
			/// Constant acceleration (also called average acceleration or trapezoid rule): gamma = 1/2, beta = 1/4.
			/// This is the most common scheme and is unconditionally stable. In this analyzer, it is used as the default.
			/// </summary>
			public void SetNewmarkParametersForConstantAcceleration()
			{
				gamma = 0.5;
				beta = 0.25;
			}

			/// <summary>
			/// Linear acceleration: gamma = 1/2, beta = 1/6. This is more accurate than the default constant acceleration,
			/// but it conditionally stable. To ensure stability, the time step must be &lt;= the critical step size = 3.464 / w
			/// = 0.551 * T, where w is the maximum natural radian frequency and T is the minimum natural period.
			/// </summary>
			public void SetNewmarkParametersForLinearAcceleration()
			{
				gamma = 0.5;
				beta = 1.0 / 6.0;
			}

			public NewmarkDynamicAnalyzer Build()
				=> new NewmarkDynamicAnalyzer(model, algebraicModel, solver, provider, childAnalyzer, timeStep, totalTime, beta, gamma);
		}
	}
}
