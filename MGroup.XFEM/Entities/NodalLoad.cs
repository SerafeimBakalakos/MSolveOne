using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.XFEM.Entities
{
    public class NodalLoad
    {
        public XNode Node { get; }

        public StructuralDof DofType { get; }

        public double Value { get; }

        public NodalLoad(XNode node, StructuralDof dofType, double value)
        {
            this.Node = node;
            this.DofType = dofType;
            this.Value = value;
        }
    }
}
