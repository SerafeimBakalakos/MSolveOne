using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.MSolve.Discretization
{
	public class GenericDofEnumerator : IElementDofEnumerator
	{
		public IReadOnlyList<IReadOnlyList<IDofType>> GetDofTypesForMatrixAssembly(IElement element) 
			=> element.ElementType.GetElementDofTypes(element);

		public IReadOnlyList<IReadOnlyList<IDofType>> GetDofTypesForDofEnumeration(IElement element) 
			=> element.ElementType.GetElementDofTypes(element);

		public IReadOnlyList<INode> GetNodesForMatrixAssembly(IElement element) => element.Nodes;

		public IMatrix GetTransformedMatrix(IMatrix matrix) => matrix;

		public double[] GetTransformedDisplacementsVector(double[] vector) => vector;

		public double[] GetTransformedForcesVector(double[] vector) => vector;
	}
}
