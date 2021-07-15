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
	public class SubdomainMatrixManagerSymmetricCsc : ISubdomainMatrixManager
	{
		private readonly SymmetricCscMatrixAssembler assembler;
		//private readonly SingleSubdomainSystem<SymmetricCscMatrix> linearSystem;

		public SubdomainMatrixManagerSymmetricCsc(ISubdomain subdomain)
		{
			assembler = new SymmetricCscMatrixAssembler(true);
			//linearSystem = new SingleSubdomainSystem<SymmetricCscMatrix>(subdomain);
		}

		public void BuildKff(ISubdomainFreeDofOrdering dofOrdering, IEnumerable<IElement> elements,
			IElementMatrixProvider matrixProvider)
			=> throw new NotImplementedException(); /*linearSystem.Matrix = assembler.BuildGlobalMatrix(dofOrdering, elements, matrixProvider);*/

		//public ILinearSystem LinearSystem => linearSystem;

		public SymmetricCscMatrix MatrixKff => throw new NotImplementedException(); /*linearSystem.Matrix;*/

		public IGlobalLinearSystem LinearSystem => throw new NotImplementedException();

		public void SetSolution(Vector solution) => throw new NotImplementedException(); /*linearSystem.SolutionConcrete = solution;*/
	}
}
