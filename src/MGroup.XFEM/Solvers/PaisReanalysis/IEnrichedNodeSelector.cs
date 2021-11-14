using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public interface IEnrichedNodeSelector
	{
		bool CanNodeBeEnriched(INode node);
	}
}
