using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.IterativeMethods.Preconditioning;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.Scaling;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

namespace MGroup.Solvers.DDM.FetiDP.Preconditioning
{
	public interface IFetiDPPreconditioner : IPreconditioner
	{
		void CalcSubdomainMatrices(int subdomainID);

		void Initialize(IComputeEnvironment environment, DistributedOverlappingIndexer lagrangeVectorIndexer,
			Func<int, SubdomainLagranges> getSubdomainLagranges, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices,
			IFetiDPScaling scaling);
	}
}
