using System.Collections.Generic;
using MGroup.MSolve.Constitutive;

namespace MGroup.FEM.Interfaces
{
	public interface IFiberFiniteElement : IFiniteElement
	{
		IConstitutiveLaw Material { get; }
		IList<IFiber> Fibers { get; }
	}
}
