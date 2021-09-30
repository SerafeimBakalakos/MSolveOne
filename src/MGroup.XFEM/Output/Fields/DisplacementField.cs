using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

//TODO: offset only vertices (which may include original nodes or anything created only for visualization) that actually lie on
//      interfaces. Furthermore these vertices should exist twice: once for each side of the interface, instead of each subcell
//      having its own instance. These should be done in ConformingMesh.
namespace MGroup.XFEM.Output.Fields
{
	public class DisplacementField
	{
		private const double offsetTol = 1E-6;

		private readonly int dimension;
		private readonly IXModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ConformingOutputMesh outMesh;

		public DisplacementField(IXModel model, IAlgebraicModel algebraicModel, ConformingOutputMesh outMesh)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.dimension = model.Dimension;
			this.outMesh = outMesh;
		}

		public Dictionary<int, double[]> CalcValuesAtVertices(IGlobalVector solution)
		{
			var outDisplacements = new Dictionary<int, double[]>();
			foreach (IXFiniteElement element in model.EnumerateElements())
			{
				IEnumerable<ConformingOutputMesh.Subcell> subtriangles = outMesh.GetSubcellsForOriginal(element);
				double[] elementVector = algebraicModel.ExtractElementVector(solution, element);
				DirichletElementLoad.ApplyBoundaryConditions(element, elementVector);
				IList<double[]> elementDisplacements = Utilities.ElementVectorToNodalVectors(element, elementVector);

				if (subtriangles.Count() == 0)
				{
					//TODO: This is incorrect for blending elements: 1) enriched dofs are not irrelevant, 2) using the
					//      FEM interpolation is not accurate, 3) they should be subdivided into subcells to accurately 
					//      calculate the displacement at their vertices and then let ParaView interpolate that (inaccurately).
					//      However some enrichments (step, ridge) do not produce blending elements (although in code, there will
					//      be elements where only some nodes are enriched). Thus the user should choose for now.
					
					Debug.Assert(outMesh.GetOutCellsForOriginal(element).Count() == 1);
					VtkCell outCell = outMesh.GetOutCellsForOriginal(element).First();
					for (int n = 0; n < element.Nodes.Count; ++n)
					{
						outDisplacements[outCell.Vertices[n].ID] = elementDisplacements[n];
					}
				}
				else
				{
					HashSet<IEnrichmentFunction> elementEnrichments = element.FindEnrichments();
					foreach (ConformingOutputMesh.Subcell subcell in subtriangles)
					{
						//TODO: Not sure what happens for 2nd order elements
						Debug.Assert(subcell.OriginalSubcell.CellType == CellType.Tri3 
							|| subcell.OriginalSubcell.CellType == CellType.Quad4
							|| subcell.OriginalSubcell.CellType == CellType.Tet4
							|| subcell.OriginalSubcell.CellType == CellType.Hexa8);

						// Find centroid of subcell
						IList<double[]> verticesNatural = subcell.OriginalSubcell.VerticesNatural;
						double[] centroid = subcell.OriginalSubcell.FindCentroidNatural();

						for (int v = 0; v < verticesNatural.Count; ++v)
						{
							// Slightly offset point towards its centroid
							double[] vertex = verticesNatural[v];
							var vertexOffset = new double[dimension];
							for (int d = 0; d < dimension; ++d)
							{
								vertexOffset[d] = vertex[d] + offsetTol * (centroid[d] - vertex[d]);
							}

							// Interpolate the nodal values, taking into account the enrichments.
							double[] u = CalcDisplacementsAtPoint(
								vertexOffset, element, elementDisplacements, elementEnrichments);
							VtkPoint vertexOut = subcell.OutVertices[v];
							outDisplacements[vertexOut.ID] = u;
						}
					}
				}
			}
			return outDisplacements;
		}

		//TODO: perhaps this can be in an element class
		private double[] CalcDisplacementsAtPoint(double[] pointNatural, IXFiniteElement element, 
			IList<double[]> elementDisplacements, IEnumerable<IEnrichmentFunction> elementEnrichments)
		{
			// Shape functions at this point
			double[] N = element.Interpolation.EvaluateFunctionsAt(pointNatural);

			// More details about this point
			var point = new XPoint(dimension);
			point.Element = element;
			point.Coordinates[CoordinateSystem.ElementNatural] = pointNatural;
			point.ShapeFunctions = N;

			// Enrichment functions at this point
			var enrichmentValues = new Dictionary<IEnrichmentFunction, double>();
			foreach (IEnrichmentFunction enrichment in elementEnrichments)
			{
				enrichmentValues[enrichment] = enrichment.EvaluateAt(point);
			}

			// u(x) = sum_over_nodes(Ni(x) * u_i) + sum_over_enriched_nodes( N_j(x) * (psi(x) - psi_j)*a_j )
			var u = new double[dimension];
			for (int n = 0; n < element.Nodes.Count; ++n)
			{
				double[] uStd = elementDisplacements[n];

				// Standard displacement
				int dof = 0;
				for (int d = 0; d < dimension; ++d)
				{
					u[d] += N[n] * uStd[dof++];
				}

				// Enriched displacement
				foreach (IEnrichmentFunction enrichment in element.Nodes[n].EnrichmentFuncs.Keys)
				{
					double psiVertex = enrichmentValues[enrichment];
					double psiNode = element.Nodes[n].EnrichmentFuncs[enrichment];
					for (int d = 0; d < dimension; ++d)
					{
						u[d] += N[n] * (psiVertex - psiNode) * uStd[dof++];
					}
				}
			}

			return u;
		}
	}
}
