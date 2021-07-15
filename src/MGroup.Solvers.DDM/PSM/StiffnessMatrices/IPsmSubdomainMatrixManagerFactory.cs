using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.PSM.Dofs;
using MGroup.Solvers.DDM.StiffnessMatrices;

namespace MGroup.Solvers.DDM.PSM.StiffnessMatrices
{
	public interface IPsmSubdomainMatrixManagerFactory
	{
		(ISubdomainMatrixManager, IPsmSubdomainMatrixManager) CreateMatrixManagers(
			ISubdomain subdomain, PsmSubdomainDofs subdomainDofs);
	}
}
