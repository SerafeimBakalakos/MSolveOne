using MGroup.MSolve.Constitutive;

namespace MGroup.FEM.Interfaces
{
	public interface IFiber
	{
		IConstitutiveLaw Material { get; }
	}
}
