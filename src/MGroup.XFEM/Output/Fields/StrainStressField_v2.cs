using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

//TODO: Remove duplication between this and StrainsStressesAtGaussPointsField
//TODO: Refactor these methods. They are too large.
namespace MGroup.XFEM.Output.Fields
{
	public class StrainStressField_v2
	{
		private const double offsetTol = 1E-6;

		private readonly int dimension;
		private readonly IXModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ConformingOutputMesh outMesh;

		public StrainStressField_v2(IXModel model, IAlgebraicModel algebraicModel, ConformingOutputMesh outMesh)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.dimension = model.Dimension;
			this.outMesh = outMesh;
		}

		public (Dictionary<int, double[]> strains, Dictionary<int, double[]> stresses) CalcTensorsAtVertices(IGlobalVector solution)
		{
			if (model.NumSubdomains != 1) throw new NotImplementedException();

			var outStrainTensors = new Dictionary<int, double[]>();
			var outStressTensors = new Dictionary<int, double[]>();
			foreach (IXCrackElement element in model.EnumerateElements())
			{
				double[] elementVector = algebraicModel.ExtractElementVector(solution, element);
				DirichletElementLoad.ApplyBoundaryConditions(element, elementVector);

				IEnumerable<ConformingOutputMesh.Subcell> subtriangles = outMesh.GetSubcellsForOriginal(element);
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
						double[] nodeNatural = element.Interpolation.NodalNaturalCoordinates[n];
						(double[] strains, double[] stresses) tensors = element.CalcStrainsStressesAt(nodeNatural, elementVector);
						VtkPoint vertexOut = outCell.Vertices[n];
						outStrainTensors[vertexOut.ID] = tensors.strains;
						outStressTensors[vertexOut.ID] = tensors.stresses;
					}
				}
				else
				{
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
							(double[] strains, double[] stresses) tensors = element.CalcStrainsStressesAt(vertexOffset, elementVector);
							VtkPoint vertexOut = subcell.OutVertices[v];
							outStrainTensors[vertexOut.ID] = tensors.strains;
							outStressTensors[vertexOut.ID] = tensors.stresses;
						}
					}
				}
			}
			return (outStrainTensors, outStressTensors);
		}
	}
}
