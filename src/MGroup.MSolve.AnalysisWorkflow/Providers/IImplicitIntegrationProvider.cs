using System.Collections.Generic;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;

//TODO: This should be called second order provider. The matrices, coefficients, etc. should be named 0th-order, 1st-order,
//      2nd-order.
//TODO: Implicit/explicit time integration logic should be defined by the analyzer and implemented by the provider, in order to
//      reuse the analyzer for problems that have a slightly different differential equation (e.g. coupled problems).
//TODO: Perhaps the providers should not hold references to the linear systems. Instead they would return vectors/matrices to
//      the analyzers (or the vectors/matrices would be passed in and overwritten).
//TODO: Rename the Get~ methods to Calculate or something similar.
namespace MGroup.MSolve.AnalysisWorkflow.Providers
{
	public interface IImplicitIntegrationProvider : IAnalyzerProvider
	{
		//TODO: This should not exist at all. The provider should return the 0th order (stiffness), 1st order (damping) and 2nd
		//      order matrices (or some matrix representations that can be combined between them and multiplied with vectors).
		void LinearCombinationOfMatricesIntoStiffness(ImplicitIntegrationCoefficients coefficients);

		void ProcessRhs(ImplicitIntegrationCoefficients coefficients, IGlobalVector rhs);
		
		IGlobalVector GetRhsFromHistoryLoad(int timeStep);
		
		//TODO: what about thermal? There is no mass matrix there. Define these as 1st order matrix coeff, 2nd order ...
		IGlobalVector MassMatrixVectorProduct(IGlobalVector vector);
		IGlobalVector DampingMatrixVectorProduct(IGlobalVector vector);

		#region These only apply to problems with acceleration e.g. dynamic elasticity. They should not be here
		//void AssignMassAccelerationHistoryLoads(int timeStep);
		//void AssignElementMassLoads();
		//void AssignMassAccelerationLoads();

		IGlobalVector GetAccelerationsOfTimeStep(int timeStep);
		#endregion

		IGlobalVector GetVelocitiesOfTimeStep(int timeStep);
	}
}
