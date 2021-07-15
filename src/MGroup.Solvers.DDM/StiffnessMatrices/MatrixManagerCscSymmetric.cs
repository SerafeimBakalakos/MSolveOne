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
	public class MatrixManagerCscSymmetric : IMatrixManager
	{
		private readonly Dictionary<int, SymmetricCscMatrixAssembler> assemblers = new Dictionary<int, SymmetricCscMatrixAssembler>();
		//private readonly Dictionary<int, SingleSubdomainSystem<SymmetricCscMatrix>> linearSystems =
			//new Dictionary<int, SingleSubdomainSystem<SymmetricCscMatrix>>();

		public MatrixManagerCscSymmetric(IModel model)
		{
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				throw new NotImplementedException();
				//linearSystems[subdomain.ID] = new SingleSubdomainSystem<SymmetricCscMatrix>(subdomain);
				//assemblers[subdomain.ID] = new SymmetricCscAssembler();
			}
		}

		public void BuildKff(int subdomainID, ISubdomainFreeDofOrdering dofOrdering, IEnumerable<IElement> elements,
			IElementMatrixProvider matrixProvider)
		{
			throw new NotImplementedException();
			//var linearSystem = linearSystems[subdomainID];
			//linearSystem.Matrix =
			//	assemblers[subdomainID].BuildGlobalMatrix(dofOrdering, linearSystem.Subdomain.Elements, matrixProvider);
		}

		public IGlobalLinearSystem GetLinearSystem(int subdomainID)
		{
			throw new NotImplementedException();
		}

		//public ILinearSystem GetLinearSystem(int subdomainID) => linearSystems[subdomainID];

		public SymmetricCscMatrix GetMatrixKff(int subdomainID) => throw new NotImplementedException(); //linearSystems[subdomainID].Matrix;

		public void SetSolution(int subdomainID, Vector solution)
		{
			throw new NotImplementedException();
			//linearSystems[subdomainID].SolutionConcrete = solution;
		}
	}
}
