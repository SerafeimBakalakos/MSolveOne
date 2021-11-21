using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class StepDofsNearModifiedNodesStrategy : IReanalysisExtraDofsStrategy
	{
		public ReanalysisRebuildingSolver Solver { get; set; }

		public HashSet<int> FindExtraModifiedCols(ISet<int> colsToAdd, ISet<int> colsToRemove)
		{
			var extraDofs = new HashSet<int>();
			Solver.FindCrackStepDofs(Solver.NodesNearModifiedNodes.NearModifiedNodes, extraDofs);
			return extraDofs;
		}
	}
}
