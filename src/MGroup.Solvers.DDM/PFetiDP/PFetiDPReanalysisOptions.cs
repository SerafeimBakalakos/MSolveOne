using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PSM;

namespace MGroup.Solvers.DDM.PFetiDP
{
	public class PFetiDPReanalysisOptions : PsmReanalysisOptions
	{
		protected PFetiDPReanalysisOptions(bool commonValueForFlags, IModifiedSubdomains modifiedSubdomains)
			: base(commonValueForFlags, modifiedSubdomains)
		{
			GlobalCoarseProblemDofs = commonValueForFlags;
		}

		public bool GlobalCoarseProblemDofs { get; set; }

		public static new PFetiDPReanalysisOptions CreateWithAllDisabled()
			=> new PFetiDPReanalysisOptions(false, new NullModifiedSubdomains());

		public static new PFetiDPReanalysisOptions CreateWithAllEnabled(IModifiedSubdomains modifiedSubdomains)
			=> new PFetiDPReanalysisOptions(true, modifiedSubdomains);

	}
}
