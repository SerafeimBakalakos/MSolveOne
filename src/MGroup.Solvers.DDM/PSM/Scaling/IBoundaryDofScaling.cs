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
		void CalcSubdomainScaling(DistributedOverlappingIndexer indexer);

		//TODOMPI: These need rework at the equation level in the distributed logic. I should probably use scaling matrices without any mapping 
		///// <summary>
		///// In theory these matrices are called Lpb.
		///// </summary>
		//IMappingMatrix GetDofMappingBoundaryClusterToSubdomain(int subdomainID);

		Dictionary<int, SparseVector> DistributeNodalLoads(Table<INode, IDofType, double> nodalLoads);

		void ScaleForceVector(int subdomainID, Vector subdomainForces); 
	}
}
