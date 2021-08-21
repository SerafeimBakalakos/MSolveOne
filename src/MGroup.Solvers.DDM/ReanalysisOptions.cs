using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Solvers.DDM.LinearSystem;

namespace MGroup.Solvers.DDM
{
	public class ReanalysisOptions
	{
		//TODO: better use static factory methods than overloaded constructors
		public ReanalysisOptions()
		{
		}

		public ReanalysisOptions(bool setAllTrue, IModifiedSubdomains modifiedSubdomains)
		{
			this.ModifiedSubdomains = modifiedSubdomains;
			IntersubdomainFreeDofs = setAllTrue;
			SubdomainFreeDofs = setAllTrue;
			SubdomainMatrix = setAllTrue;
		}

		public bool IntersubdomainFreeDofs { get; set; } = false;

		public IModifiedSubdomains ModifiedSubdomains { get; set; } = new NullModifiedSubdomains();

		public bool SubdomainFreeDofs { get; set; } = false;

		public bool SubdomainMatrix { get; set; } = false;

	}
}
