using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using System.Collections.Concurrent;
using MGroup.Environments;
using MGroup.Solvers.DDM.PSM.Dofs;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.DataStructures;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PSM.Vectors;
using System.Diagnostics;

namespace MGroup.Solvers.DDM.PSM.Scaling
{
	public class HomogeneousScaling : IBoundaryDofScaling
	{
		private readonly IComputeEnvironment environment;
		private readonly Func<int, PsmSubdomainDofs> getSubdomainDofs;

		private readonly ConcurrentDictionary<int, double[]> inverseMultiplicities = new ConcurrentDictionary<int, double[]>();

		public HomogeneousScaling(IComputeEnvironment environment, Func<int, PsmSubdomainDofs> getSubdomainDofs)
		{
			this.environment = environment;
			this.getSubdomainDofs = getSubdomainDofs;
		}

		public void CalcScalingMatrices(DistributedOverlappingIndexer boundaryDofIndexer)
		{
			Action<int> calcSubdomainScaling = subdomainID =>
			{
				int numBoundaryDofs = getSubdomainDofs(subdomainID).DofsBoundaryToFree.Length;

				var subdomainW = new double[numBoundaryDofs];
				int[] multiplicites = boundaryDofIndexer.GetLocalComponent(subdomainID).Multiplicities;
				for (int i = 0; i < numBoundaryDofs; ++i)
				{
					subdomainW[i] = 1.0 / multiplicites[i];
				}

				//DofTable boundaryDofs = dofSeparator.GetSubdomainDofOrderingBoundary(subdomainID);
				//foreach ((INode node, IDofType dof, int idx) in boundaryDofs)
				//{
				//	subdomainW[idx] = 1.0 / node.SubdomainsDictionary.Count;
				//}

				inverseMultiplicities[subdomainID] = subdomainW;

				//BooleanMatrixRowsToColumns Lb = dofSeparator.GetDofMappingBoundaryClusterToSubdomain(subdomain.ID);
				//var Lpb = new ScalingMatrixRowMajor(Lb.NumRows, Lb.NumColumns, Lb.RowsToColumns, subdomainW);
				//dofMappingBoundaryClusterToSubdomain[subdomain.ID] = Lpb;
			};
			environment.DoPerNode(calcSubdomainScaling);
		}

		public void ScaleBoundaryRhsVector(int subdomainID, Vector boundaryRhsVector)
		{
			double[] coefficients = inverseMultiplicities[subdomainID];
			Debug.Assert(boundaryRhsVector.Length == coefficients.Length);
			for (int i = 0; i < coefficients.Length; i++)
			{
				boundaryRhsVector[i] *= coefficients[i];
			}
		}
	}
}
