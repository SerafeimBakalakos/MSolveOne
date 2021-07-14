using MGroup.MSolve.AnalysisWorkflow;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;

using System.Collections.Generic;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.MSolve.Solution;
using System;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.NumericalAnalyzers.NonLinear
{
	public class NonLinearModelUpdater : INonLinearModelUpdater
	{
		private readonly IModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ElementInternalRhsProvider rhsProvider = new ElementInternalRhsProvider();


		public NonLinearModelUpdater(IModel model, IAlgebraicModel algebraicModel)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
		}

		//TODO: I suggest splitting this into 2 methods. One for updating the elements/materials and one for calculating the internal rhs
		public IGlobalVector GetRhsFromSolution(IGlobalVector solution)
		{
			// First update the state of the elements
			algebraicModel.DoPerElement(model.EnumerateElements, element =>
			{
				double[] elementDisplacements = algebraicModel.ExtractElementVector(solution, element);
				DirichletElementLoad.ApplyBoundaryConditions(element, elementDisplacements);
				element.ElementType.CalculateStresses(element, elementDisplacements);
			});

			// Then calculate the internal rhs vector
			IGlobalVector internalRhs = algebraicModel.CreateZeroVector();
			algebraicModel.AddToGlobalVector(model.EnumerateElements, internalRhs, rhsProvider);
			return internalRhs;
		}

		public void ScaleConstraints(double scalingFactor) => model.ScaleConstraints(scalingFactor);

		public void UpdateState() => model.SaveMaterialState();
	}
}
