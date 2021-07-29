using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public interface IFetiDPCoarseProblemFactory
	{
		IFetiDPCoarseProblem CreateCoarseProblem(
			IComputeEnvironment environment, SubdomainTopology subdomainTopology,
			Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices);
	}
}
