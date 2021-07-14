using System;
using System.Collections.Generic;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;

using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.NumericalAnalyzers.NonLinear
{
	/// <summary>
	/// Subdomain state update class that accounts for non zero initial conditions (displacements).
	/// Authors: Gerasimos Sotiropoulos
	/// </summary>
	public class NonLinearModelUpdaterWithInitialConditions : INonLinearModelUpdater
	{
		private readonly IModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ElementInternalRhsProvider rhsProvider = new ElementInternalRhsProvider();

		public NonLinearModelUpdaterWithInitialConditions(IModel model, IAlgebraicModel algebraicModel)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
		}

		//public void ResetState()
		//{
		//	this.subdomain.ClearMaterialStresses();
		//}

		public void UpdateState() => model.SaveMaterialState();

		public IGlobalVector GetRHSFromSolutionWithInitialDisplacemntsEffect(IGlobalVector solution, Dictionary<int, INode> boundaryNodes,
		Dictionary<int, Dictionary<IDofType, double>> initialConvergedBoundaryDisplacements, Dictionary<int, Dictionary<IDofType, double>> totalBoundaryDisplacements,
		int nIncrement, int totalIncrements)
		{
			// First update the state of the elements
			algebraicModel.DoPerElement(model.EnumerateElements, element =>
			{
				double[] localSolution = algebraicModel.ExtractElementVector(solution, element);
				ImposePrescribedDisplacementsWithInitialConditionSEffect(element, localSolution, boundaryNodes, initialConvergedBoundaryDisplacements, totalBoundaryDisplacements, nIncrement, totalIncrements);
				element.ElementType.CalculateStresses(element, localSolution);
			});

			// Then calculate the internal rhs vector
			IGlobalVector internalRhs = algebraicModel.CreateZeroVector();
			algebraicModel.AddToGlobalVector(model.EnumerateElements, internalRhs, rhsProvider);
			return internalRhs;
		}


		public IGlobalVector GetRhsFromSolution(IGlobalVector solution)
		{
			// First update the state of the elements
			algebraicModel.DoPerElement(model.EnumerateElements, element =>
			{
				double[] localSolution = algebraicModel.ExtractElementVector(solution, element);
				DirichletElementLoad.ApplyBoundaryConditions(element, localSolution);
				element.ElementType.CalculateStresses(element, localSolution);
			});

			// Then calculate the internal rhs vector
			IGlobalVector internalRhs = algebraicModel.CreateZeroVector();
			algebraicModel.AddToGlobalVector(model.EnumerateElements, internalRhs, rhsProvider);
			return internalRhs;
		}

		//public double[] CalculateElementIncrementalConstraintDisplacements(IElement element, double constraintScalingFactor, ISubdomainFreeDofOrdering SubdomainFreeDofOrdering)//QUESTION: would it be maybe more clear if we passed the constraintsDictionary as argument??
		//{
		//	var elementNodalDisplacements = new double[SubdomainFreeDofOrdering.CountElementDofs(element)];
		//	SubdomainConstrainedDofOrderingBase.ApplyConstraintDisplacements(element, elementNodalDisplacements, this.subdomain.Constraints);
		//	return elementNodalDisplacements;
		//}

		public void ImposePrescribedDisplacementsWithInitialConditionSEffect(IElement element, double[] localSolution, Dictionary<int, INode> boundaryNodes,
			Dictionary<int, Dictionary<IDofType, double>> initialConvergedBoundaryDisplacements, Dictionary<int, Dictionary<IDofType, double>> totalBoundaryDisplacements,
			int nIncrement, int totalIncrements)
		{
			var elementDOFTypes = element.ElementType.DofEnumerator.GetDofTypesForMatrixAssembly(element);
			var matrixAssemblyNodes = element.ElementType.DofEnumerator.GetNodesForMatrixAssembly(element);
			int iElementMatrixColumn = 0;
			for (int j = 0; j < elementDOFTypes.Count; j++)
			{
				INode nodeColumn = matrixAssemblyNodes[j];
				int nodalDofsNumber = elementDOFTypes[j].Count;
				if (boundaryNodes.ContainsKey(nodeColumn.ID))
				{
					Dictionary<IDofType, double> nodalConvergedDisplacements = initialConvergedBoundaryDisplacements[nodeColumn.ID];
					Dictionary<IDofType, double> nodalTotalDisplacements = totalBoundaryDisplacements[nodeColumn.ID];
					int positionOfDofInNode = 0;
					foreach (IDofType doftype1 in elementDOFTypes[j])
					{
						if (nodalConvergedDisplacements.ContainsKey(doftype1))
						{
							localSolution[iElementMatrixColumn + positionOfDofInNode] = nodalConvergedDisplacements[doftype1] + (nodalTotalDisplacements[doftype1] - nodalConvergedDisplacements[doftype1]) * ((double)nIncrement / (double)totalIncrements);
							// TODO: this can be done faster: create a dictionary<...,dictionary> with the difference of the two values and use that and precalculate coefficient for scaling
						}
						positionOfDofInNode += 1;
					}
				}
				iElementMatrixColumn += nodalDofsNumber;
			}

		}

		public void ImposePrescribed_d_DisplacementsWithInitialConditionSEffect(IElement element, double[] localSolution, Dictionary<int, INode> boundaryNodes,
			Dictionary<int, Dictionary<IDofType, double>> initialConvergedBoundaryDisplacements, Dictionary<int, Dictionary<IDofType, double>> totalBoundaryDisplacements,
			int nIncrement, int totalIncrements)
		{

			var elementDOFTypes = element.ElementType.DofEnumerator.GetDofTypesForMatrixAssembly(element);
			var matrixAssemblyNodes = element.ElementType.DofEnumerator.GetNodesForMatrixAssembly(element);
			int iElementMatrixColumn = 0;
			for (int j = 0; j < elementDOFTypes.Count; j++)
			{
				INode nodeColumn = matrixAssemblyNodes[j];
				int nodalDofsNumber = elementDOFTypes[j].Count;
				if (boundaryNodes.ContainsKey(nodeColumn.ID))
				{
					Dictionary<IDofType, double> nodalConvergedDisplacements = initialConvergedBoundaryDisplacements[nodeColumn.ID];
					Dictionary<IDofType, double> nodalTotalDisplacements = totalBoundaryDisplacements[nodeColumn.ID];
					int positionOfDofInNode = 0;
					foreach (IDofType doftype1 in elementDOFTypes[j])
					{
						if (nodalConvergedDisplacements.ContainsKey(doftype1))
						{
							localSolution[iElementMatrixColumn + positionOfDofInNode] = (nodalTotalDisplacements[doftype1] - nodalConvergedDisplacements[doftype1]) * ((double)nIncrement / (double)totalIncrements);
							// 1) den vazoume mono (1/increments) alla (nIncrement/increments) dioti metaxu aftwn twn nIncrements den exei mesolavhsei save sta material ths mikroklimakas
							// TODO: this can be done faster: create a dictionary<...,dictionary> with the difference of the two values and use that and precalculate coefficient for scaling
						}
						positionOfDofInNode += 1;
					}
				}
				iElementMatrixColumn += nodalDofsNumber;
			}

		}

		public void ScaleConstraints(double scalingFactor) => model.ScaleConstraints(scalingFactor);
	}
}
