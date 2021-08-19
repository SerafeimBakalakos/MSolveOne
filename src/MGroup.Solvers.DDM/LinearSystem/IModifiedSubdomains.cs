using System;
using System.Collections.Generic;
using System.Text;

//TODO: Perhaps these should be done by ISubdomain itself
namespace MGroup.Solvers.DDM.LinearSystem
{
	public interface IModifiedSubdomains
	{
		bool IsConnectivityModified(int subdomainID);

		bool IsStiffnessModified(int subdomainID);
	}
}
