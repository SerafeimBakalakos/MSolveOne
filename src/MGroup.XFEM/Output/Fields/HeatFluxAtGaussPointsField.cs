using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Interpolation;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.LinearAlgebra.Distributed;

//TODO: Also calculate heat flux at nodes. It needs averaging over the elements. It also needs to specify the material since it 
//      is not explicitly stored as in elements.
namespace MGroup.XFEM.Output.Fields
{
	public class HeatFluxAtGaussPointsField
	{
		private readonly XModel<IXMultiphaseElement> model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly bool ignoreIntersectedElements;

		public HeatFluxAtGaussPointsField(XModel<IXMultiphaseElement> model, IAlgebraicModel algebraicModel, 
			bool ignoreIntersectedElements = false)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.ignoreIntersectedElements = ignoreIntersectedElements;
		}

		public Dictionary<double[], double[]> CalcValuesAtVertices(IGlobalVector solution)
		{
			var result = new Dictionary<double[], double[]>();
			foreach (IXThermalElement element in model.Elements.Values)
			{
				if (ignoreIntersectedElements && (element.InteractingDiscontinuities.Count > 0))
				{
					continue;
				}

				(IReadOnlyList<GaussPoint> gaussPoints, IReadOnlyList<ThermalMaterial> materials)
					= element.GetMaterialsForBulkIntegration();
				double[] nodalTemperatures = Utilities.ExtractNodalTemperatures(algebraicModel, element, solution);
				for (int i = 0; i < gaussPoints.Count; ++i)
				{
					GaussPoint pointNatural = gaussPoints[i];
					EvalInterpolation evalInterpolation =
						element.Interpolation.EvaluateAllAt(element.Nodes, pointNatural.Coordinates);
					double[] coordsCartesian = 
						Utilities.TransformNaturalToCartesian(evalInterpolation.ShapeFunctions, element.Nodes);
					var point = new XPoint(coordsCartesian.Length);
					point.Coordinates[CoordinateSystem.ElementNatural] = pointNatural.Coordinates;
					point.Element = element;
					point.ShapeFunctions = evalInterpolation.ShapeFunctions;
					double[] gradientTemperature =
						CalcTemperatureGradientAt(point, evalInterpolation, element, nodalTemperatures);

					double conductivity = materials[i].ThermalConductivity;
					for (int d = 0; d < gradientTemperature.Length; d++)
					{
						gradientTemperature[d] *= -conductivity;
					}
					result[coordsCartesian] = gradientTemperature;
				}
			}
			return result;
		}

		public static double[] CalcTemperatureGradientAt(XPoint point, EvalInterpolation evalInterpolation,
			IXFiniteElement element, double[] nodalTemperatures)
		{
			int dimension = evalInterpolation.ShapeGradientsCartesian.NumColumns;
			var gradient = new double[dimension];
			int idx = 0;
			for (int n = 0; n < element.Nodes.Count; ++n)
			{
				// Standard temperatures
				double stdTi = nodalTemperatures[idx++];
				for (int i = 0; i < dimension; ++i)
				{
					gradient[i] += evalInterpolation.ShapeGradientsCartesian[n, i] * stdTi;
				}

				// Eniched temperatures
				foreach (IEnrichmentFunction enrichment in element.Nodes[n].EnrichmentFuncs.Keys)
				{
					double psiVertex = enrichment.EvaluateAt(point);
					double psiNode = element.Nodes[n].EnrichmentFuncs[enrichment];
					double enrTij = nodalTemperatures[idx++];

					for (int i = 0; i < dimension; ++i)
					{
						gradient[i] += evalInterpolation.ShapeGradientsCartesian[n, i] * (psiVertex - psiNode) * enrTij;
					}
				}
			}
			return gradient;
		}
	}
}
