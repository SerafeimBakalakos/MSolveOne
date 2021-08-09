using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Meshes.Structured;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Output.Fields
{
	public class TemperaturePostProcessor
	{
		private readonly ICartesianMesh mesh;
		private readonly XModel<IXMultiphaseElement> model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly double[] dx;

		public TemperaturePostProcessor(ICartesianMesh mesh, XModel<IXMultiphaseElement> model, IAlgebraicModel algebraicModel)
		{
			this.mesh = mesh;
			this.model = model;
			this.algebraicModel = algebraicModel;
			dx = new double[mesh.Dimension];
			for (int d = 0; d < mesh.Dimension; ++d)
			{
				dx[d] = (mesh.MaxCoordinates[d] - mesh.MinCoordinates[d]) / mesh.NumElements[d];
			}
		}

		public List<double> CalcTemperatureAt(IList<double[]> cartesianPoints, IGlobalVector solution)
		{
			var results = new List<double>(cartesianPoints.Count); 
			foreach (double[] cartesianPoint in cartesianPoints)
			{
				(int elementID, double[] naturalCoords) = FindElementContaining(cartesianPoint);
				IXMultiphaseElement element = model.Elements[elementID];

				IList<double[]> nodalTemperatures = Utilities.ExtractNodalTemperatures(algebraicModel, element, solution);
				var point = new XPoint(naturalCoords.Length);
				point.Element = element;
				point.Coordinates[CoordinateSystem.ElementNatural] = naturalCoords;
				point.ShapeFunctions = element.Interpolation.EvaluateFunctionsAt(naturalCoords);
				//double[] coordsCartesian = Utilities.TransformNaturalToCartesian(point.ShapeFunctions, element.Nodes);
				results.Add(CalcTemperatureAt(point, element, nodalTemperatures));
			}
			return results;
		}

		public static double CalcTemperatureAt(XPoint point, IXFiniteElement element, IList<double[]> nodalTemperatures)
		{
			double sum = 0.0;
			for (int n = 0; n < element.Nodes.Count; ++n)
			{
				double[] Tn = nodalTemperatures[n];
				int idx = 0;

				// Standard temperatures
				sum += point.ShapeFunctions[n] * Tn[idx++];

				// Eniched temperatures
				foreach (IEnrichmentFunction enrichment in element.Nodes[n].EnrichmentFuncs.Keys)
				{
					double psiVertex = enrichment.EvaluateAt(point);
					double psiNode = element.Nodes[n].EnrichmentFuncs[enrichment];
					sum += point.ShapeFunctions[n] * (psiVertex - psiNode) * Tn[idx++];
				}
			}
			return sum;
		}

		private (int elementID, double[] naturalCoords) FindElementContaining(double[] cartesianCoords)
		{
			int dim = mesh.Dimension;
			var elementIdx = new int[dim];
			var naturalCoords = new double[dim];
			for (int d = 0; d < dim; ++d)
			{
				// For a given axis:
				// Let x be the global cartesian coordinate of a point along this axis.
				// Let xMin be the lower bound of x.
				// Let e be the index of the element that contains x along this axis.
				// Let xi be the isoparametric coordinate of element e, that corresponds to x.
				// Let dx be the length of each element along this axis.
				// Let xStart = xMin + e*dx and xEnd = xMin + (e+1)*dx be the global cartesian coordinates of the start 
				// and end point of element e.
				// Then x = xMin + xStart * 1/2 * (1-xi) + xEnd * 1/2 * (1+xi) = ... = xMin + e*dx + dx * (1+xi)/2 
				// <=> x = xMin + xStart + dx * (1+xi)/2 <=> xi = 2*(x-xMin)/dx -2*e -1 
				// where dx * (1+xi)/2 belongs in [0, dx]
				double x = cartesianCoords[d];
				double xMin = mesh.MinCoordinates[d];
				elementIdx[d] = (int)Math.Floor((x - xMin) / dx[d]);
				naturalCoords[d] = 2 * (x - xMin) / dx[d] - 2 * elementIdx[d] - 1;
			}

			return (mesh.GetElementID(elementIdx), naturalCoords);
		}
	}
}
