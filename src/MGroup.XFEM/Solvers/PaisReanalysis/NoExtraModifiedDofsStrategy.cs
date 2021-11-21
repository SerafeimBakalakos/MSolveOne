using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class NoExtraModifiedDofsStrategy : IReanalysisExtraDofsStrategy
	{
		public ReanalysisRebuildingSolver Solver { get; set; }

		public HashSet<int> FindExtraModifiedCols(ISet<int> colsToAdd, ISet<int> colsToRemove)
		{
			return new HashSet<int>();
		}
	}
}
