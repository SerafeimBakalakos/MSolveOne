using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;

//TODO: Should this be in the Problems project?
namespace MGroup.Constitutive.Structural
{
	public class ElementStructuralStiffnessProvider : IElementMatrixProvider
	{
		public IMatrix Matrix(IElement element) => element.ElementType.StiffnessMatrix(element);
	}
}
