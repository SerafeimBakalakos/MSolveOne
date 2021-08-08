using System;

using MGroup.FEM.Entities;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.MSolve.AnalysisWorkflow.Providers;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;

//TODO: Usually the LinearSystem is passed in, but for GetRHSFromHistoryLoad() it is stored as a field. Decide on one method.
//TODO: I am not too fond of the provider storing global sized matrices.
namespace MGroup.Constitutive.Thermal
{
	public class ProblemThermal : IImplicitIntegrationProvider, IStaticProvider, INonLinearProvider
	{
		private IGlobalMatrix capacity, conductivity;
		private readonly IModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ISolver solver;
		private ElementConductivityProvider conductivityProvider = new ElementConductivityProvider();
		private ElementCapacityProvider capacityProvider = new ElementCapacityProvider();
		private readonly IElementMatrixPredicate rebuildConductivityPredicate = new MaterialModifiedElementMarixPredicate();

		public ProblemThermal(IModel model, IAlgebraicModel algebraicModel, ISolver solver)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.solver = solver;
		}

		private IGlobalMatrix Capacity
		{
			get
			{
				if (capacity == null) BuildCapacity();
				return capacity;
			}
		}

		private IGlobalMatrix Conductivity
		{
			get
			{
				if (conductivity == null) BuildConductivity();
				//else RebuildConductivityMatrices();
				return conductivity;
			}
		}

		private void BuildConductivity()
		{
			conductivity = algebraicModel.BuildGlobalMatrix(model.EnumerateElements, conductivityProvider);
		}
		
		private void RebuildConductivity()
		{
			algebraicModel.RebuildGlobalMatrixPartially(
				conductivity, model.EnumerateElements, conductivityProvider, rebuildConductivityPredicate);
		}

		private void BuildCapacity() 
			=> capacity = algebraicModel.BuildGlobalMatrix(model.EnumerateElements, capacityProvider);

		#region IAnalyzerProvider Members
		public void ClearMatrices()
		{
			capacity = null;
			conductivity = null;
		}

		public void Reset()
		{
			// TODO: Check if we should clear material state - (goat) removed that, seemed erroneous
			//foreach (ISubdomain subdomain in model.Subdomains)
			//	foreach (IElement element in subdomain.Elements)
			//		element.ElementType.ClearMaterialState();

			conductivity = null;
			capacity = null;
		}
		#endregion

		#region IImplicitIntegrationProvider Members

		public void LinearCombinationOfMatricesIntoStiffness(ImplicitIntegrationCoefficients coefficients)
		{
			// The effective matrix should not overwrite the conductivity matrix. 
			// In a dynamic analysis that is not purely implicit we need the conductivity matrix.
			solver.LinearSystem.Matrix = Conductivity.LinearCombination(coefficients.Stiffness, Capacity, coefficients.Mass);
		}

		public void ProcessRhs(ImplicitIntegrationCoefficients coefficients, IGlobalVector rhs)
		{
			// Method intentionally left empty.
		}

		public IGlobalVector GetAccelerationsOfTimeStep(int timeStep)
		{
			throw new InvalidOperationException("This is does not make sense in explicit methods for first order equations");
		}

		public IGlobalVector GetVelocitiesOfTimeStep(int timeStep)
		{
			throw new InvalidOperationException("This is not needed in explicit methods for first order equations");
		}

		public IGlobalVector GetRhsFromHistoryLoad(int timeStep)
		{//TODO: Remove type casting
			solver.LinearSystem.RhsVector.Clear(); //TODO: this is also done by model.AssignLoads()
			AssignRhs();
			//var femModel = (Model)model;
			//solver.LinearSystem.AddToGlobalVector(femModel.EnumerateNodalLoads, solver.LinearSystem.RhsVector);
			//femModel.TimeStep = timeStep;
			//solver.LinearSystem.AddToGlobalVector(femModel.EnumerateTransientNodalLoads, solver.LinearSystem.RhsVector);

			IGlobalVector result = solver.LinearSystem.RhsVector.Copy();
			return result;
		}

		public IGlobalVector MassMatrixVectorProduct(IGlobalVector vector)
		{
			IGlobalVector result = algebraicModel.CreateZeroVector();
			Capacity.MultiplyVector(vector, result);
			return result;
		}

		//TODO: Ok this is weird. These methods should be named Second/First/ZeroOrderCoefficientTimesVector()
		public IGlobalVector DampingMatrixVectorProduct(IGlobalVector vector)
		{
			IGlobalVector result = algebraicModel.CreateZeroVector();
			Conductivity.MultiplyVector(vector, result);
			return result;
		}

		#endregion

		#region IStaticProvider Members

		public void CalculateMatrix()
		{
			if (conductivity == null) BuildConductivity();
			solver.LinearSystem.Matrix = conductivity;
		}
		#endregion

		#region INonLinearProvider Members

		public double CalculateRhsNorm(IGlobalVector rhs) => rhs.Norm2();

		public void ProcessInternalRhs(IGlobalVector solution, IGlobalVector rhs) { }

		#endregion

		public void AssignRhs()
		{// TODO: Remove type casting
			solver.LinearSystem.RhsVector.Clear();
			algebraicModel.AddToGlobalVector(model.EnumerateNodalLoads, solver.LinearSystem.RhsVector);
		}
	}
}
