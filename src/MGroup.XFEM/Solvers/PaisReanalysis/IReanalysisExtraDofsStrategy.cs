using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public interface IReanalysisExtraDofsStrategy
	{
		ReanalysisRebuildingSolver Solver { get; set; }

		HashSet<int> FindExtraModifiedCols(ISet<int> colsToAdd, ISet<int> colsToRemove); 
	}
}
