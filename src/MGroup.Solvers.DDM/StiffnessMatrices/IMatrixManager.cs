using System.Collections.Generic;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.DDM.StiffnessMatrices
{
	public interface IMatrixManager
	{
		void BuildKff(int subdomainID, ISubdomainFreeDofOrdering dofOrdering, IEnumerable<IElement> elements,
			IElementMatrixProvider matrixProvider);

		IGlobalLinearSystem GetLinearSystem(int subdomainID);

		//TODO: Refactor this
		void SetSolution(int subdomainID, Vector solution);
	}
}
