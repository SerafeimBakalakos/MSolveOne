using System;
using System.Collections.Generic;
using System.Text;

using MGroup.MSolve.Solution.LinearSystem;

namespace MGroup.MSolve.Solution.AlgebraicModel
{
	/// <summary>
	/// This component is responsible for the linear algebra representation, namely global vectors and matrices, of the physical 
	/// model. It is also responsible for conversions between the physical and algebraic representations.  
	/// </summary>
	public interface IAlgebraicModel : IGlobalMatrixAssembler, IGlobalVectorAssembler, IVectorValueExtractor, IElementIterator 
	{
		IGlobalLinearSystem LinearSystem { get; }

		void OrderDofs();
	}
}
