using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Solvers.DDM.LinearSystem
{
	public class NullSubdomainModification : ISubdomainModification
	{
		public bool IsConnectivityModified(int subdomainID) => true;

		public bool IsStiffnessModified(int subdomainID) => true;
	}
}
