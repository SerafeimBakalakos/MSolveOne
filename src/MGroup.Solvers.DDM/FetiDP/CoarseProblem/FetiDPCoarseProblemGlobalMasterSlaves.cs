using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemGlobalMasterSlaves : FetiDPCoarseProblemGlobalBase
	{
		private readonly Func<int, FetiDPSubdomainDofs> getSubdomainDofs;

		public FetiDPCoarseProblemGlobalMasterSlaves(IComputeEnvironment environment, 
			IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix,
			Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices)
			: base(environment, coarseProblemMatrix, getSubdomainMatrices)
		{
			this.getSubdomainDofs = getSubdomainDofs;
		}

		protected override Dictionary<int, IntDofTable> GatherSubdomainCornerDofs()
		{
			return environment.GatherToMasterNode(subdomainID => getSubdomainDofs(subdomainID).DofOrderingCorner);
		}

		protected override Dictionary<int, IMatrix> GatherSubdomainMatricesScc()
		{
			return environment.GatherToMasterNode(subdomainID => getSubdomainMatrices(subdomainID).SchurComplementOfRemainderDofs);
		}

		protected override Dictionary<int, Vector> GatherSubdomainVectors(IDictionary<int, Vector> coarseProblemRhs)
		{
			return environment.GatherToMasterNode(subdomainID => coarseProblemRhs[subdomainID]);
		}

		protected override void ScatterSubdomainVectors(
			Dictionary<int, Vector> localVectors, IDictionary<int, Vector> remoteVectors)
		{
			Dictionary<int, Vector> scatteredVectors = environment.ScatterFromMasterNode(localVectors);
			foreach (int s in scatteredVectors.Keys)
			{
				remoteVectors[s] = scatteredVectors[s];
			}
		}

		public class Factory : IFetiDPCoarseProblemFactory
		{
			private readonly IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix;

			public Factory(IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix)
			{
				this.coarseProblemMatrix = coarseProblemMatrix;
			}

			public IFetiDPCoarseProblem CreateCoarseProblem(
				IComputeEnvironment environment, IModel model, SubdomainTopology subdomainTopology,
				Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices)
			{
				return new FetiDPCoarseProblemGlobalMasterSlaves(environment, coarseProblemMatrix, getSubdomainDofs, 
					getSubdomainMatrices);
			}
		}
	}
}
