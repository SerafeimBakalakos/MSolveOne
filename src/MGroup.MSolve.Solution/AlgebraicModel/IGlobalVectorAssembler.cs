using System;
using System.Collections.Generic;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;

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
