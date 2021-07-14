using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.AlgebraicModel;
using MGroup.LinearAlgebra.Matrices;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
	public class HomogeneousScaling : IPrimalScaling
	{
		private readonly IModel model;
		private readonly DistributedAlgebraicModel<Matrix> algebraicModel;
		private readonly IGlobalFreeDofOrdering dofOrdering;

		public HomogeneousScaling(IModel model, DistributedAlgebraicModel<Matrix> algebraicModel)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
		}

		public Dictionary<int, SparseVector> DistributeNodalLoads(Table<INode, IDofType, double> nodalLoads)
		{
			Func<int, SparseVector> calcSubdomainForces = subdomainID =>
			{
				ISubdomain subdomain = model.GetSubdomain(subdomainID);
				ISubdomainFreeDofOrdering dofOrdering = algebraicModel.DofOrdering.SubdomainDofOrderings[subdomain.ID];
				DofTable freeDofs = dofOrdering.FreeDofs;

				//TODO: I go through every node and ignore the ones that are not loaded. 
				//		It would be better to directly access the loaded ones.
				var nonZeroLoads = new SortedDictionary<int, double>();
				foreach (INode node in subdomain.Nodes)
				{
					bool isLoaded = nodalLoads.TryGetDataOfRow(node, out IReadOnlyDictionary<IDofType, double> loadsOfNode);
					if (!isLoaded) continue;

					foreach (var dofLoadPair in loadsOfNode)
					{
						int freeDofIdx = freeDofs[node, dofLoadPair.Key];
						nonZeroLoads[freeDofIdx] = dofLoadPair.Value / node.SubdomainsDictionary.Count;
					}
				}

				return SparseVector.CreateFromDictionary(dofOrdering.NumFreeDofs, nonZeroLoads);
			};

			var results = new Dictionary<int, SparseVector>();
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				results[subdomain.ID] = calcSubdomainForces(subdomain.ID);
			}
			return results;
		}
	}
}
