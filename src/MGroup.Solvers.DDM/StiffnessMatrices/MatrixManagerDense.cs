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
	public class MatrixManagerDense : IMatrixManager
	{
		private readonly Dictionary<int, DenseMatrixAssembler> assemblers = new Dictionary<int, DenseMatrixAssembler>();
		//private readonly Dictionary<int, SingleSubdomainSystem<Matrix>> linearSystems =
			//new Dictionary<int, SingleSubdomainSystem<Matrix>>();

		public MatrixManagerDense(IModel model)
		{
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				throw new NotImplementedException();
				//linearSystems[subdomain.ID] = new SingleSubdomainSystem<Matrix>(subdomain);
				//assemblers[subdomain.ID] = new DenseMatrixAssembler();
			}
		}

		public void BuildKff(int subdomainID, ISubdomainFreeDofOrdering dofOrdering, IEnumerable<IElement> elements,
			IElementMatrixProvider matrixProvider)
		{
			throw new NotImplementedException();
			//var linearSystem = linearSystems[subdomainID];
			//linearSystem.Matrix = assemblers[subdomainID].BuildGlobalMatrix(dofOrdering,
			//	linearSystem.Subdomain.Elements, matrixProvider);
		}

		//public ILinearSystem GetLinearSystem(int subdomainID) => linearSystems[subdomainID];
		IGlobalLinearSystem IMatrixManager.GetLinearSystem(int subdomainID)
		{
			throw new NotImplementedException();
		}

		public Matrix GetMatrixKff(int subdomainID) => throw new NotImplementedException();/*linearSystems[subdomainID].Matrix;*/

		public void SetSolution(int subdomainID, Vector solution)
		{
			throw new NotImplementedException();
			//linearSystems[subdomainID].SolutionConcrete = solution;
		}

		
	}
}
