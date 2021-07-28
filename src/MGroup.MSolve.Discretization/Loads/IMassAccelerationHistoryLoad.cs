using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.MSolve.Discretization.Loads
{
    public interface IMassAccelerationHistoryLoad
    {
        IDofType DOF { get; }
        double this[int currentTimeStep] { get; }
    }
}
