using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices.Builders;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class LimitedDofsNearModifiedDofsStategy : IReanalysisExtraDofsStrategy
	{
		public ReanalysisRebuildingSolver Solver { get; set; }

		public HashSet<int> FindExtraModifiedCols(ISet<int> colsToAdd, ISet<int> colsToRemove)
		{
			var basicModifiedCols = new HashSet<int>(colsToAdd);
			basicModifiedCols.UnionWith(colsToRemove);

			// Nodes with dofs with modified stiffness or near them
			HashSet<XNode> modifiedNodes = AllDofsNearModifiedDofsStrategy.FindNodesNearModifiedDofs(Solver);

			// Dofs with unmodified diagonal entries, but modified super-diagonal entries
			IntDofTable dofOrdering = Solver.AlgebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			DokSymmetric matrix = Solver.LinearSystem.Matrix.SingleMatrix.DokMatrix;
			var extraCols = new HashSet<int>();
			foreach (XNode node in modifiedNodes)
			{
				List<int> nearbyDofs = FindNearbyDofs(node);
				foreach (var pair in dofOrdering.GetDataOfRow(node.ID))
				{
					int dofID = pair.Key;
					int col = pair.Value;
					if (basicModifiedCols.Contains(col))
					{
						// We only care about dofs with unmodified stiffness now
						continue;
					}

					Dictionary<int, double> wholeColumn = matrix.RawColumns[col];
					if (wholeColumn.Count == 1 && wholeColumn[col] == 1.0)
					{
						// Inactive dof
						continue;
					}

					if (MustUpdateNearModifiedDof(node.ID, dofID, col, nearbyDofs, basicModifiedCols))
					{
						extraCols.Add(col);
					}
				}
			}
			return extraCols;
		}

		private List<int> FindNearbyDofs(XNode node)
		{
			var nearbyNodes = new HashSet<XNode>();
			foreach (IXFiniteElement element in node.ElementsDictionary.Values)
			{
				nearbyNodes.UnionWith(element.Nodes);
			}

			IntDofTable dofOrdering = Solver.AlgebraicModel.SubdomainFreeDofOrdering.FreeDofs;
			var nearbyDofs = new List<int>();
			foreach (XNode otherNode in nearbyNodes)
			{
				nearbyDofs.AddRange(dofOrdering.GetValuesOfRow(otherNode.ID));
			}
			return nearbyDofs;
		}

		private bool MustUpdateNearModifiedDof(int nodeID, int dofID, int col, List<int> nearbyDofs, ISet<int> basicModifiedCols)
		{
			int j = col;
			foreach (int i in nearbyDofs)
			{
				if (basicModifiedCols.Contains(i))
				{
					// At this point we have an unmodified dof with a neighboring modified one
					if (i < j)
					{
						// The coupling stiffness entry Aij belongs to column j and has been modified. 
						// Thus the whole column needs to be modified.
						return true;
					}
				}
			}
			return false;
		}
	}
}
