using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;

namespace MGroup.Multiscale.Interfaces
{
	public interface IReferenceVolumeElement
	{
		void ApplyBoundaryConditions();
		IMatrixView CalculateKinematicRelationsMatrix(ISubdomain subdomain);
		double CalculateRveVolume();
	}
}
