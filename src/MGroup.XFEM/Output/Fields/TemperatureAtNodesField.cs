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
			IDofType[] dofsPerNode = { ThermalDof.Temperature };
			foreach (XNode node in model.Nodes.Values)
			{
				double[] nodalValues = algebraicModel.ExtractNodalValues(solution, node, dofsPerNode);
				result[node.Coordinates] = nodalValues[0];
			}
			return result;
		}
	}
}
