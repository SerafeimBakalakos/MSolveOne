using System;
using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.DDM.StiffnessMatrices
{
	public class SubdomainMatrixManagerCsr : ISubdomainMatrixManager
	{
		private readonly CsrMatrixAssembler assembler;
		//private readonly SingleSubdomainSystem<CsrMatrix> linearSystem;

		public SubdomainMatrixManagerCsr(ISubdomain subdomain, bool isSymmetric)
		{
			assembler = new CsrMatrixAssembler(isSymmetric, true);
			//linearSystem = new SingleSubdomainSystem<CsrMatrix>(subdomain);
		}

		public void BuildKff(ISubdomainFreeDofOrdering dofOrdering, IEnumerable<IElement> elements,
			IElementMatrixProvider matrixProvider)
			=> throw new NotImplementedException();/*linearSystem.Matrix = assembler.BuildGlobalMatrix(dofOrdering, elements, matrixProvider);*/

		//public ILinearSystem LinearSystem => linearSystem;
		public IGlobalLinearSystem LinearSystem => throw new NotImplementedException();

		public CsrMatrix MatrixKff => throw new NotImplementedException();/*linearSystem.Matrix;*/

		public void SetSolution(Vector solution) => throw new NotImplementedException(); /*linearSystem.SolutionConcrete = solution;*/
	}
}
