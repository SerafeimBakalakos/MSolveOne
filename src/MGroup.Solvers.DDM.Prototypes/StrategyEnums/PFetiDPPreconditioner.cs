using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Solvers.DDM.Prototypes.StrategyEnums
{
	public enum PFetiDPPreconditioner
	{
		OriginalMonolithic, 
		OriginalDistributive,
		DistributedInterfaceOriginalCoarse,
		DistributedAll
	}
}
