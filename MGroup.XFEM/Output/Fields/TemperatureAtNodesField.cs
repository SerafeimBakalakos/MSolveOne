using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.LinearAlgebra.Distributed;

namespace MGroup.XFEM.Output.Fields
{
	public class TemperatureAtNodesField
	{
		private readonly XModel<IXMultiphaseElement> model;
		private readonly IAlgebraicModel algebraicModel;

		public TemperatureAtNodesField(XModel<IXMultiphaseElement> model, IAlgebraicModel algebraicModel)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
		}

		public Dictionary<double[], double> CalcValuesAtVertices(IGlobalVector solution)
		{
			var result = new Dictionary<double[], double>();
			foreach (XNode node in model.Nodes.Values)
			{
				IDofType dof = ThermalDof.Temperature;
				try //TODO: This should be a feature offered by IAlgebraicModel: find displacements/etc of a node for a list of dofs
				{
					result[node.Coordinates] = algebraicModel.ExtractSingleValue(solution, node, dof);
				}
				catch (KeyNotFoundException)
				{
					result[node.Coordinates] = node.Constraints.Find(con => con.DOF == dof).Amount;
				}
			}
			return result;
		}
	}
}
