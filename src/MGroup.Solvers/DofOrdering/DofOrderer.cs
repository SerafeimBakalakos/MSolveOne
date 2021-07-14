using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization;

using MGroup.Solvers.DofOrdering.Reordering;

namespace MGroup.Solvers.DofOrdering
{
    /// <summary>
    /// Orders the unconstrained freedom degrees of each subdomain and the shole model. Also applies any reordering and other 
    /// optimizations.
    /// </summary>
    public class DofOrderer : IDofOrderer
    {
        //TODO: this should also be a strategy, so that I could have caching with fallbacks, in case of insufficient memor.
        private readonly bool cacheElementToSubdomainDofMaps = true; 
        private readonly bool doOptimizationsIfSingleSubdomain = true; // No idea why someone would want this to be false.
        private readonly IFreeDofOrderingStrategy freeOrderingStrategy;
        private readonly IDofReorderingStrategy reorderingStrategy;

        public DofOrderer(IFreeDofOrderingStrategy freeOrderingStrategy, IDofReorderingStrategy reorderingStrategy,
            bool doOptimizationsIfSingleSubdomain = true, bool cacheElementToSubdomainDofMaps = true)
        {
            this.freeOrderingStrategy = freeOrderingStrategy;
            this.reorderingStrategy = reorderingStrategy;
            this.doOptimizationsIfSingleSubdomain = doOptimizationsIfSingleSubdomain;
            this.cacheElementToSubdomainDofMaps = cacheElementToSubdomainDofMaps;
        }

        public IGlobalFreeDofOrdering OrderFreeDofs(IModel model)
        {
            if (doOptimizationsIfSingleSubdomain && (model.NumSubdomains == 1))
            {
                ISubdomain subdomain = model.EnumerateSubdomains().First();

                // Order subdomain dofs
                (int numSubdomainFreeDofs, DofTable subdomainFreeDofs) = freeOrderingStrategy.OrderSubdomainDofs(subdomain);
                ISubdomainFreeDofOrdering subdomainOrdering;
                if (cacheElementToSubdomainDofMaps)
                {
                    subdomainOrdering = new SubdomainFreeDofOrderingCaching(numSubdomainFreeDofs, subdomainFreeDofs);
                }
                else subdomainOrdering = new SubdomainFreeDofOrderingGeneral(numSubdomainFreeDofs, subdomainFreeDofs);

                // Reorder subdomain dofs
                reorderingStrategy.ReorderDofs(subdomain, subdomainOrdering);

                // Order global dofs
                return new GlobalFreeDofOrderingSingle(subdomain, subdomainOrdering);
            }
            else
            {
                // Order subdomain dofs
                var subdomainOrderings = new Dictionary<int, ISubdomainFreeDofOrdering>(model.NumSubdomains);
                foreach (ISubdomain subdomain in model.EnumerateSubdomains())
                {
                    (int numSubdomainFreeDofs, DofTable subdomainFreeDofs) = freeOrderingStrategy.OrderSubdomainDofs(subdomain);
                    ISubdomainFreeDofOrdering subdomainOrdering;
                    if (cacheElementToSubdomainDofMaps) subdomainOrdering = new SubdomainFreeDofOrderingCaching(
                        numSubdomainFreeDofs, subdomainFreeDofs);
                    else subdomainOrdering = new SubdomainFreeDofOrderingGeneral(numSubdomainFreeDofs, subdomainFreeDofs);
                    subdomainOrderings.Add(subdomain.ID, subdomainOrdering);

                    // Reorder subdomain dofs
                    reorderingStrategy.ReorderDofs(subdomain, subdomainOrdering);
                }

                // Order global dofs
                (int numGlobalFreeDofs, DofTable globalFreeDofs) = freeOrderingStrategy.OrderGlobalDofs(model);
                return new GlobalFreeDofOrderingGeneral(numGlobalFreeDofs, globalFreeDofs, subdomainOrderings);
            }
        }
    }
}
