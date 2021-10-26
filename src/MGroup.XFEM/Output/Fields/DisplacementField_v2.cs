using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Discretization.Dofs;
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
	public class DisplacementField_v2
	{
		private const double offsetTol = 1E-6;

		private readonly int dimension;
		private readonly IXModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ConformingContinuousMesh outMesh;

		public DisplacementField_v2(IXModel model, IAlgebraicModel algebraicModel, ConformingContinuousMesh outMesh)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.dimension = model.Dimension;
			this.outMesh = outMesh;
		}

		public Dictionary<int, double[]> CalcValuesAtVertices(IGlobalVector solution)
		{
			var outDisplacements = new Dictionary<int, double[]>();
			var elementVectors = new Dictionary<int, double[]>();
			foreach (int v in outMesh.Vertices.Keys)
			{
				if (outMesh.VerticesCoordinates[v].OriginalNodeID != int.MinValue)
				{
					// This is a node. Just read its value from the solution.
					XNode node = model.Nodes[outMesh.VerticesCoordinates[v].OriginalNodeID];
					IDofType[] dofs = model.Dimension == 2 ?
						new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY } :
						new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ };
					outDisplacements[v] = algebraicModel.ExtractNodalValues(solution, node, dofs);
				}
				else
				{
					// This is an intersection point. Interpolate its value from an element
					var pointDisplacements = new Dictionary<int, double[]>();
					foreach (var pair in outMesh.VerticesCoordinates[v].NaturalCoords)
					{
						IXFiniteElement element = pair.Key;
						double[] naturalCoords = pair.Value;

						// Isolate element displacements (std, enr)
						bool isElementProcessed = elementVectors.TryGetValue(element.ID, out double[] elementVector);
						if (!isElementProcessed)
						{
							elementVector = algebraicModel.ExtractElementVector(solution, element);
							DirichletElementLoad.ApplyBoundaryConditions(element, elementVector);
						}

						pointDisplacements[element.ID] = CalcDisplacementsAtPoint(naturalCoords, element, elementVector);
						outDisplacements[v] = AverageDisplacementsAtIntersection(pointDisplacements);
					}
				}
			}

			return outDisplacements;
		}

		//TODO: This should be done in the element class, like strains and stresses
		private double[] CalcDisplacementsAtPoint(double[] pointNatural, IXFiniteElement element, double[] elementVector)
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
			foreach (IEnrichmentFunction enrichment in element.FindEnrichments())
			{
				enrichmentValues[enrichment] = enrichment.EvaluateAt(point);
			}

			// u(x) = sum_over_nodes(Ni(x) * u_i) + sum_over_enriched_nodes( N_j(x) * (psi(x) - psi_j)*a_j )
			var u = new double[dimension];
			int dof = 0;
			for (int n = 0; n < element.Nodes.Count; ++n)
			{
				// Standard displacement
				for (int d = 0; d < dimension; ++d)
				{
					u[d] += N[n] * elementVector[dof++];
				}

				// Enriched displacement
				foreach (IEnrichmentFunction enrichment in element.Nodes[n].EnrichmentFuncs.Keys)
				{
					double psiVertex = enrichmentValues[enrichment];
					double psiNode = element.Nodes[n].EnrichmentFuncs[enrichment];
					for (int d = 0; d < dimension; ++d)
					{
						u[d] += N[n] * (psiVertex - psiNode) * elementVector[dof++];
					}
				}
			}

			return u;
		}

		private double[] AverageDisplacementsAtIntersection(Dictionary<int, double[]> elementDisplacementsAtPoint)
		{
			int numEntries = model.Dimension;
			int numElements = elementDisplacementsAtPoint.Count;
			var result = new double[numEntries];
			foreach (double[] u in elementDisplacementsAtPoint.Values)
			{
				for (int i = 0; i < numEntries; ++i)
				{
					result[i] += u[i] / numElements;
				}
			}
			return result;
		}
	}
}
