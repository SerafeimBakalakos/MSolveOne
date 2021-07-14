using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution.LinearSystem;

namespace MGroup.MSolve.Solution.AlgebraicModel
{
	public interface IGlobalVectorAssembler
	{
		void AddToGlobalVector(Func<int, IEnumerable<IElement>> accessElements, IGlobalVector vector, 
			IElementVectorProvider vectorProvider);

		void AddToGlobalVector(Func<int, IEnumerable<IElementLoad>> acessLoads, IGlobalVector vector);


		void AddToGlobalVector(Func<int, IEnumerable<INodalLoad>> accessLoads, IGlobalVector vector);


		void AddToGlobalVector(IEnumerable<IAllNodeLoad> loads, IGlobalVector vector);

		IGlobalVector CreateZeroVector();
	}
}
