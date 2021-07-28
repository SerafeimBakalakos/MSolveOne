using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Solvers.DofOrdering.Reordering;

namespace MGroup.Solvers.DofOrdering
{
	/// <summary>
	/// Orders the unconstrained freedom degrees of a subdomain. Optionally applies reordering and other optimizations.
	/// </summary>
	public class DofOrderer : IDofOrderer
	{
		//TODO: this should also be a strategy, so that I could have caching with fallbacks, in case of insufficient memor.
		private readonly bool cacheElementToSubdomainDofMaps = true; 
		private readonly IFreeDofOrderingStrategy freeOrderingStrategy;
		private readonly IDofReorderingStrategy reorderingStrategy;

		public DofOrderer(IFreeDofOrderingStrategy freeOrderingStrategy, IDofReorderingStrategy reorderingStrategy,
			bool cacheElementToSubdomainDofMaps = true)
		{
			this.freeOrderingStrategy = freeOrderingStrategy;
			this.reorderingStrategy = reorderingStrategy;
			this.cacheElementToSubdomainDofMaps = cacheElementToSubdomainDofMaps;
		}

		public ISubdomainFreeDofOrdering OrderFreeDofs(ISubdomain subdomain, ActiveDofs allDofs)
		{
			// Order subdomain dofs
			(int numSubdomainFreeDofs, IntDofTable subdomainFreeDofs) = 
				freeOrderingStrategy.OrderSubdomainDofs(subdomain, allDofs);
			ISubdomainFreeDofOrdering subdomainOrdering;
			if (cacheElementToSubdomainDofMaps)
			{
				subdomainOrdering = new SubdomainFreeDofOrderingCaching(numSubdomainFreeDofs, subdomainFreeDofs, allDofs);
			}
			else
			{
				subdomainOrdering = new SubdomainFreeDofOrderingGeneral(numSubdomainFreeDofs, subdomainFreeDofs, allDofs);
			}

			// Reorder subdomain dofs
			reorderingStrategy.ReorderDofs(subdomain, subdomainOrdering);

			return subdomainOrdering;
		}
	}
}
