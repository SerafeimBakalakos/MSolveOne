using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.XFEM.Output.Writers
{
	public interface IResultsWriter
	{
		void WriteResults(IAlgebraicModel algebraicModel, IGlobalVector solution);
	}
}
