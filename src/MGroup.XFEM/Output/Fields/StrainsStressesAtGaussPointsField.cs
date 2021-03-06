using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;

namespace MGroup.XFEM.Output.Fields
{
	public class StrainsStressesAtGaussPointsField
	{
		private readonly IXModel model;
		private readonly IAlgebraicModel algebraicModel;

		public StrainsStressesAtGaussPointsField(IXModel model, IAlgebraicModel algebraicModel)
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
			foreach (IXFiniteElement element in model.EnumerateElements())
			{
				IList<double[]> elementDisplacements = Utilities.ElementVectorToNodalVectors(element,
						algebraicModel.ExtractElementVector(solution, element));
				HashSet<IEnrichmentFunction> elementEnrichments = element.FindEnrichments();
				foreach (GaussPoint gp in element.BulkIntegrationPoints)
				{
					XPoint point = StrainStressField.PreparePoint(gp.Coordinates, element);
					double[] coordsCartesian =
						Utilities.TransformNaturalToCartesian(point.ShapeFunctions, element.Nodes);
					(double[] strains, double[] stresses) = StrainStressField.CalcStrainsStressesAt(
						point, element, elementDisplacements, elementEnrichments);
					allStrains[coordsCartesian] = strains;
					allStresses[coordsCartesian] = stresses;
				}
			}
			return (allStrains, allStresses);
		}
	}
}
