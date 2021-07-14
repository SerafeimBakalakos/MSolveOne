using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;

namespace MGroup.MSolve.Solution.AlgebraicModel
{
	public class MaterialModifiedElementMarixPredicate : IElementMatrixPredicate
	{
		public bool MustBuildMatrixForElement(IElement element) => element.ElementType.MaterialModified;

		public void ProcessElementAfterBuildingMatrix(IElement element) => element.ElementType.ResetMaterialModified();

		public void ProcessElementAfterNotBuildingMatrix(IElement element) { } // Do nothing
	}
}
