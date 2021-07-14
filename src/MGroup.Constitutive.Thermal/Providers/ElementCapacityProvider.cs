using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;

namespace MGroup.Constitutive.Thermal
{
    public class ElementCapacityProvider : IElementMatrixProvider
    {
        public IMatrix Matrix(IElement element) => element.ElementType.MassMatrix(element);
    }
}
