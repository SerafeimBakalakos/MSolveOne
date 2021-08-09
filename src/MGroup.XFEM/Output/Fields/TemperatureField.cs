using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.LinearAlgebra.Distributed;

namespace MGroup.XFEM.Output.Fields
{
	public class TemperatureField
	{
		private readonly XModel<IXMultiphaseElement> model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ConformingOutputMesh outMesh;

		public TemperatureField(XModel<IXMultiphaseElement> model, IAlgebraicModel algebraicModel, ConformingOutputMesh outMesh)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.outMesh = outMesh;
		}

		public Dictionary<int, double> CalcValuesAtVertices(IGlobalVector systemSolution)
		{
			var outTemperatures = new Dictionary<int, double>();
			foreach (IXFiniteElement element in model.Elements.Values)
			{

				IEnumerable<ConformingOutputMesh.Subcell> subtriangles = outMesh.GetSubcellsForOriginal(element);
				if (subtriangles.Count() == 0)
				{
					IList<double[]> nodalTemperatures = Utilities.ExtractNodalTemperatures(algebraicModel, element, systemSolution);
					Debug.Assert(outMesh.GetOutCellsForOriginal(element).Count() == 1);
					VtkCell outCell = outMesh.GetOutCellsForOriginal(element).First();
					for (int n = 0; n < element.Nodes.Count; ++n)
					{
						outTemperatures[outCell.Vertices[n].ID] = nodalTemperatures[n][0]; // ignore enriched dofs in blending elements
					}
				}
				else
				{
					IList<double[]> nodalTemperatures = Utilities.ExtractNodalTemperatures(algebraicModel, element, systemSolution);
					foreach (ConformingOutputMesh.Subcell subcell in subtriangles)
					{
						Debug.Assert(subcell.OutVertices.Count == 3 || subcell.OutVertices.Count == 4); //TODO: Not sure what happens for 2nd order elements

						// We must interpolate the nodal values, taking into account the enrichements.
						double[] temperatureAtVertices = CalcTemperatureFieldInSubtriangle(element,
							subcell.OriginalSubcell, nodalTemperatures);

						for (int v = 0; v < subcell.OutVertices.Count; ++v)
						{
							VtkPoint vertexOut = subcell.OutVertices[v];
							outTemperatures[vertexOut.ID] = temperatureAtVertices[v];
						}
					}
				}
			}
			return outTemperatures;
		}

		private double[] CalcTemperatureFieldInSubtriangle(IXFiniteElement element, IElementSubcell subcell,
			IList<double[]> nodalTemperatures)
		{
			// Evaluate shape functions
			var shapeFunctionsAtVertices = new List<double[]>(subcell.VerticesNatural.Count);
			for (int v = 0; v < subcell.VerticesNatural.Count; ++v)
			{
				double[] vertex = subcell.VerticesNatural[v];
				shapeFunctionsAtVertices.Add(element.Interpolation.EvaluateFunctionsAt(vertex));
			}

			// Locate centroid
			double[] centroidNatural = subcell.FindCentroidNatural();
			var centroid = new XPoint(centroidNatural.Length);
			centroid.Element = element;
			centroid.Coordinates[CoordinateSystem.ElementNatural] = centroidNatural;
			centroid.ShapeFunctions = element.Interpolation.EvaluateFunctionsAt(centroidNatural);

			// Evaluate enrichment functions at triangle centroid and assume it also holds for its vertices
			var enrichments = new HashSet<IEnrichmentFunction>();
			foreach (XNode node in element.Nodes) enrichments.UnionWith(node.EnrichmentFuncs.Keys);
			var enrichmentValues = new Dictionary<IEnrichmentFunction, double>();
			foreach (IEnrichmentFunction enrichment in enrichments)
			{
				enrichmentValues[enrichment] = enrichment.EvaluateAt(centroid);
				//enrichmentValues[enrichment] = EvaluateFunctionsAtSubtriangleVertices(
				//    element, shapeFunctionsAtVertices, shapeFunctionsAtCentroid);
			}

			// t(x) = sum_over_nodes(Ni(x) * t_i) + sum_over_enriched_nodes( N_j(x) * (psi(x) - psi_j)*a_j )
			var temperaturesAtVertices = new double[subcell.VerticesNatural.Count];
			for (int v = 0; v < subcell.VerticesNatural.Count; ++v)
			{
				double[] N = shapeFunctionsAtVertices[v];
				double sum = 0.0;
				for (int n = 0; n < element.Nodes.Count; ++n)
				{
					double[] Tn = nodalTemperatures[n];
					int idx = 0;

					// Standard temperatures
					sum += N[n] * Tn[idx++];

					// Eniched temperatures
					foreach (IEnrichmentFunction enrichment in element.Nodes[n].EnrichmentFuncs.Keys)
					{
						double psiVertex = enrichmentValues[enrichment];
						double psiNode = element.Nodes[n].EnrichmentFuncs[enrichment];
						sum += N[n] * (psiVertex - psiNode) * Tn[idx++];
					}
				}
				temperaturesAtVertices[v] = sum;
			}
			return temperaturesAtVertices;
		}
	}
}
