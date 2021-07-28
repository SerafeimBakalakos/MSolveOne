using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.FEM.Entities
{
	public class ElementMassAccelerationLoad
	{
		public Element Element { get; set; }

		public IDofType DOF { get; set; }

		public double Amount { get; set; }
	}
}
