using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Distributed.Overlapping;

namespace MGroup.Solvers.DDM.FetiDP.InterfaceProblem
{
	public interface IFetiDPInterfaceProblemMatrix : ILinearTransformation
	{
		void Calculate(DistributedOverlappingIndexer lagrangeVectorIndexer);
	}
}
