using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.MSolve.Discretization.Loads
{
	public class MassAccelerationLoad : IAllNodeLoad
	{
		public IDofType DOF { get; set; }
		public double Amount { get; set; }
	}
}
