using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;

//TODO: Perhaps this should be done by IModel
namespace MGroup.MSolve.Solution.AlgebraicModel
{
	public interface IElementIterator
	{
		void DoPerElement(Func<int, IEnumerable<IElement>> accessElements, Action<IElement> elementAction);
	}
}
