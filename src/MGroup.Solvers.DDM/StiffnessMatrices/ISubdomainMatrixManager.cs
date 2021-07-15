using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.DDM.StiffnessMatrices
{
	public interface ISubdomainMatrixManager
	{
		void BuildKff(ISubdomainFreeDofOrdering dofOrdering, IEnumerable<IElement> elements, 
			IElementMatrixProvider matrixProvider);

		IGlobalLinearSystem LinearSystem { get; }

		//TODO: Refactor this
		void SetSolution(Vector solution);
	}
}
