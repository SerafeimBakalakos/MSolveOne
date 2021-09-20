using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Integration;

namespace MGroup.XFEM.Output.Fields
{
	public class StrainsStressesAtGaussPointsField_v2
	{
		private readonly IXModel model;
		private readonly IAlgebraicModel algebraicModel;

		public StrainsStressesAtGaussPointsField_v2(IXModel model, IAlgebraicModel algebraicModel)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
		}

		public (Dictionary<double[], double[]> strains, Dictionary<double[], double[]> stresses) 
			CalcTensorsAtPoints(IGlobalVector solution)
		{
			if (model.NumSubdomains != 1) throw new NotImplementedException();

			var allStrains = new Dictionary<double[], double[]>();
			var allStresses = new Dictionary<double[], double[]>();
			foreach (IXCrackElement element in model.EnumerateElements())
			{
				double[] elementVector = algebraicModel.ExtractElementVector(solution, element);
				DirichletElementLoad.ApplyBoundaryConditions(element, elementVector);
				
				foreach (GaussPoint gp in element.BulkIntegrationPoints)
				{
					double[] shapeFunctions = element.Interpolation.EvaluateFunctionsAt(gp.Coordinates);
					double[] coordsCartesian = Utilities.TransformNaturalToCartesian(shapeFunctions, element.Nodes);
					(double[] strains, double[] stresses) = element.CalcStrainsStressesAt(gp.Coordinates, elementVector);
					allStrains[coordsCartesian] = strains;
					allStresses[coordsCartesian] = stresses;
				}
			}
			return (allStrains, allStresses);
		}
	}
}
