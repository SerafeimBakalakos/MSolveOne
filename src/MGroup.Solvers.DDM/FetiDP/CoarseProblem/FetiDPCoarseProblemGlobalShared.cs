using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemGlobalShared : FetiDPCoarseProblemGlobalBase
	{
		private readonly IModel model;
		private readonly Func<int, FetiDPSubdomainDofs> getSubdomainDofs;

		public FetiDPCoarseProblemGlobalShared(IComputeEnvironment environment, IModel model, 
			IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix,
			Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices)
			: base(environment, coarseProblemMatrix, getSubdomainMatrices)
		{
			this.model = model;
			this.getSubdomainDofs = getSubdomainDofs;
		}

		protected override Dictionary<int, IntDofTable> GatherSubdomainCornerDofs()
		{
			var subdomainCornerDofs = new Dictionary<int, IntDofTable>();
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				subdomainCornerDofs[subdomain.ID] = getSubdomainDofs(subdomain.ID).DofOrderingCorner;
			}
			return subdomainCornerDofs;
		}

		protected override Dictionary<int, IMatrix> GatherSubdomainMatricesScc()
		{
			var subdomainMatricesScc = new Dictionary<int, IMatrix>();
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				subdomainMatricesScc[subdomain.ID] = getSubdomainMatrices(subdomain.ID).SchurComplementOfRemainderDofs;
			}
			return subdomainMatricesScc;
		}

		protected override Dictionary<int, Vector> GatherSubdomainVectors(IDictionary<int, Vector> coarseProblemRhs)
		{
			var localVectors = new Dictionary<int, Vector>();
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				localVectors[s] = coarseProblemRhs[s];
			}
			return localVectors;
		}

		protected override void ScatterSubdomainVectors(
			Dictionary<int, Vector> localVectors, IDictionary<int, Vector> remoteVectors)
		{
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				remoteVectors[s] = localVectors[s];
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
				return new FetiDPCoarseProblemGlobalShared(environment, model, coarseProblemMatrix, getSubdomainDofs, 
					getSubdomainMatrices);
			}
		}
	}
}
