using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.DDM.StiffnessMatrices
{
	public class SubdomainMatrixManagerDense : ISubdomainMatrixManager
	{
		private readonly DenseMatrixAssembler assembler;
		//private readonly SingleSubdomainSystem<Matrix> linearSystem;

		public SubdomainMatrixManagerDense(ISubdomain subdomain)
		{
			throw new NotImplementedException();
			//assembler = new DenseMatrixAssembler();
			//linearSystem = new SingleSubdomainSystem<Matrix>(subdomain);
		}

		public void BuildKff(ISubdomainFreeDofOrdering dofOrdering, IEnumerable<IElement> elements,
			IElementMatrixProvider matrixProvider)
			=> throw new NotImplementedException();/* linearSystem.Matrix = assembler.BuildGlobalMatrix(dofOrdering, elements, matrixProvider);*/

		//public ILinearSystem LinearSystem => linearSystem;

		public Matrix MatrixKff => throw new NotImplementedException(); //linearSystem.Matrix;

		public IGlobalLinearSystem LinearSystem => throw new NotImplementedException();

		public void SetSolution(Vector solution) => throw new NotImplementedException(); /*linearSystem.SolutionConcrete = solution;*/
	}
}
