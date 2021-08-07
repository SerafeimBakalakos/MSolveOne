using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Phases;


//TODO: This needs splitting up. At the very least separate thermal from structural
namespace MGroup.XFEM.Output.Fields
{
	public static class Utilities
	{
		public static Dictionary<int, double> CalcBulkSizeOfEachPhase(XModel<IXMultiphaseElement> physicalModel,
			PhaseGeometryModel geometryModel)
		{
			var bulkSizes = new Dictionary<int, double>();
			foreach (IPhase phase in geometryModel.Phases.Values) bulkSizes[phase.ID] = 0.0;

			foreach (IXMultiphaseElement element in physicalModel.Elements.Values)
			{
				if ((element.ConformingSubcells == null) || (element.ConformingSubcells.Length == 0))
				{
					System.Diagnostics.Debug.Assert(element.Phases.Count == 1);
					IPhase phase = element.Phases.First();
					double elementBulkSize = element.CalcBulkSizeCartesian();
					bulkSizes[phase.ID] += elementBulkSize;
				}
				else
				{
					foreach (IElementSubcell subcell in element.ConformingSubcells)
					{
						double[] centroidNatural = subcell.FindCentroidNatural();
						var centroid = new XPoint(centroidNatural.Length);
						centroid.Coordinates[CoordinateSystem.ElementNatural] = centroidNatural;
						centroid.Element = element;
						centroid.ShapeFunctions =
							element.Interpolation.EvaluateFunctionsAt(centroid.Coordinates[CoordinateSystem.ElementNatural]);
						IPhase phase = element.FindPhaseAt(centroid);
						centroid.PhaseID = phase.ID;

						(_, double subcellBulk) = subcell.FindCentroidAndBulkSizeCartesian(element);

						bulkSizes[phase.ID] += subcellBulk;
					}
				}
			}

			return bulkSizes;
		}

		internal static IList<double[]> ExtractElementDisplacements(IAlgebraicModel algebraicModel, IXFiniteElement element, 
			IGlobalVector solution)
		{
			var nodalDisplacements = new List<double[]>(element.Nodes.Count);
			IReadOnlyList<IReadOnlyList<IDofType>> nodalDofs = element.GetElementDofTypes(element);
			for (int n = 0; n < element.Nodes.Count; ++n)
			{
				XNode node = element.Nodes[n];
				double[] displacementsOfNode = new double[nodalDofs[n].Count];
				for (int i = 0; i < displacementsOfNode.Length; ++i)
				{
					IDofType dof = nodalDofs[n][i];
					try //TODO: This should be a feature offered by IAlgebraicModel: find displacements/etc of a node for a list of dofs
					{
						displacementsOfNode[i] = algebraicModel.ExtractSingleValue(solution, node, dof);
					}
					catch (KeyNotFoundException)
					{
						displacementsOfNode[i] = node.Constraints.Find(con => con.DOF == dof).Amount;
					}
				}
				nodalDisplacements.Add(displacementsOfNode);
			}
			return nodalDisplacements.ToArray();
		}

		internal static double[] ExtractNodalTemperatures(IAlgebraicModel algebraicModel, IXFiniteElement element, 
			IGlobalVector solution)
		{
			var nodalTemperatures = new List<double>(element.Nodes.Count);
			IReadOnlyList<IReadOnlyList<IDofType>> nodalDofs = element.GetElementDofTypes(element);
			for (int n = 0; n < element.Nodes.Count; ++n)
			{
				XNode node = element.Nodes[n];
				foreach (IDofType dof in nodalDofs[n])
				{
					try //TODO: This should be a feature offered by IAlgebraicModel: find displacements/etc of a node for a list of dofs
					{
						nodalTemperatures.Add(algebraicModel.ExtractSingleValue(solution, node, dof));
					}
					catch (KeyNotFoundException)
					{
						nodalTemperatures.Add(node.Constraints.Find(con => con.DOF == dof).Amount);
					}
				}
			}
			return nodalTemperatures.ToArray();
		}

		internal static double[] TransformNaturalToCartesian(double[] shapeFunctionsAtPoint, IReadOnlyList<XNode> nodes)
		{
			int dim = nodes[0].Coordinates.Length;
			var result = new double[dim];

			for (int i = 0; i < nodes.Count; ++i)
			{
				for (int d = 0; d < dim; ++d)
				{
					result[d] += shapeFunctionsAtPoint[i] * nodes[i].Coordinates[d];
				}
			}
			return result;
		}
	}
}
