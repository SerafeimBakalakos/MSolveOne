using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.Mappings;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.DataStructures;

namespace MGroup.Solvers.DDM.PSM.Scaling
{
	public interface IBoundaryDofScaling
	{
		void CalcScalingMatrices(DistributedOverlappingIndexer indexer);

		void ScaleBoundaryRhsVector(int subdomainID, Vector subdomainForces); 
	}
}
