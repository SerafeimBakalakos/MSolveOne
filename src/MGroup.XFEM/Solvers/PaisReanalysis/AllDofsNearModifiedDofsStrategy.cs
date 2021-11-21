using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices.Builders;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class AllDofsNearModifiedDofsStrategy : IReanalysisExtraDofsStrategy
	{
		public ReanalysisRebuildingSolver Solver { get; set; }

		public HashSet<int> FindExtraModifiedCols(ISet<int> colsToAdd, ISet<int> colsToRemove)
		{
			var basicModifiedCols = new HashSet<int>(colsToAdd);
			basicModifiedCols.UnionWith(colsToRemove);

			// Nodes with dofs with modified stiffness or near them
			HashSet<XNode> modifiedNodes = FindNodesNearModifiedDofs(Solver);

			var extraCols = new HashSet<int>();
			IntDofTable dofOrdering = Solver.AlgebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			foreach (XNode node in modifiedNodes)
			{
				foreach (int col in dofOrdering.GetValuesOfRow(node.ID))
				{
					if (!basicModifiedCols.Contains(col))
					{
						extraCols.Add(col);
					}
				}
			}

			return extraCols;
		}

		internal static HashSet<XNode> FindNodesNearModifiedDofs(ReanalysisRebuildingSolver solver)
		{
			var modifiedNodes = new HashSet<XNode>();
			modifiedNodes.UnionWith(solver.NewStepNodes.NewCrackStepNodes);
			modifiedNodes.UnionWith(solver.NewTipNodes.NewCrackTipNodes);
			modifiedNodes.UnionWith(solver.PreviousTipNodes.PreviousCrackTipNodes);
			modifiedNodes.UnionWith(solver.StepNodesWithModifiedLevelSet.StepNodesWithModifiedLevelSets);

			var modifiedElements = new HashSet<IXFiniteElement>();
			foreach (XNode node in modifiedNodes)
			{
				modifiedElements.UnionWith(node.ElementsDictionary.Values);
			}
			foreach (IXFiniteElement element in modifiedElements)
			{
				modifiedNodes.UnionWith(element.Nodes);
			}
			return modifiedNodes;
		}
	}
}
