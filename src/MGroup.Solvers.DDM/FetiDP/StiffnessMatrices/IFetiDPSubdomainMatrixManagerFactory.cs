using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.StiffnessMatrices;

namespace MGroup.Solvers.DDM.FetiDP.StiffnessMatrices
{
	public interface IFetiDPSubdomainMatrixManagerFactory
	{
		(ISubdomainMatrixManager, IFetiDPSubdomainMatrixManager) CreateMatrixManagers(ISubdomain subdomain, FetiDPSubdomainDofs dofSeparator);
	}
}
