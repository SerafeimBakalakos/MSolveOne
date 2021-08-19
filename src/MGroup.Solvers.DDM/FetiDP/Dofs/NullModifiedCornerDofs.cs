using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Solvers.DDM.LinearSystem;

namespace MGroup.Solvers.DDM.FetiDP.Dofs
{
	public class NullModifiedCornerDofs : IModifiedCornerDofs
	{
		public bool AreGlobalCornerDofsModified => true;

		public bool AreSubdomainCornerDofsModified(int subdomainID) => true;

		public void Update(IModifiedSubdomains modifiedSubdomains)
		{
		}
	}
}
