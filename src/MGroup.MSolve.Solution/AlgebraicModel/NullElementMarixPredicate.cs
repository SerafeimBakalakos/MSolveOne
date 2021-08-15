using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;

namespace MGroup.MSolve.Solution.AlgebraicModel
{
	public class NullElementMarixPredicate : IElementMatrixPredicate
	{
		public bool MustBuildMatrixForElement(IElement element) => true;

		public void ProcessElementAfterBuildingMatrix(IElement element) { }

		public void ProcessElementAfterNotBuildingMatrix(IElement element) { } // Do nothing
	}
}
