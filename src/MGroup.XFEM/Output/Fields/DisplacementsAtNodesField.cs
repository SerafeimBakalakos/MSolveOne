using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.LinearAlgebra.Distributed;

namespace MGroup.XFEM.Output.Fields
{
	public class DisplacementsAtNodesField
	{
		private readonly XModel<IXMultiphaseElement> model;
		private readonly IAlgebraicModel algebraicModel;

		public DisplacementsAtNodesField(XModel<IXMultiphaseElement> model, IAlgebraicModel algebraicModel)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
		}

		public Dictionary<double[], double[]> CalcValuesAtVertices(IGlobalVector solution)
		{
			IDofType[] dofsPerNode;
			if (model.Dimension == 2)
			{
				dofsPerNode = new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY };
			}
			else
			{
				dofsPerNode = new IDofType[] 
				{ 
					StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ 
				};
			}

			var result = new Dictionary<double[], double[]>();
			foreach (XNode node in model.Nodes.Values)
			{
				result[node.Coordinates] = algebraicModel.ExtractNodalValues(solution, node, dofsPerNode);
			}
			return result;
		}
	}
}
