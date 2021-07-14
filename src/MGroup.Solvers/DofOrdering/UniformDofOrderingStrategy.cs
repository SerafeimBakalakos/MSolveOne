using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.DataStructures;

namespace MGroup.Solvers.DofOrdering
{
    /// <summary>
    /// Free dofs are assigned global / subdomain indices in a node major fashion: The dofs of the first node are numbered, then 
    /// the dofs of the second node, etc. Note that the dofs of each node are assumed to be the same and supplied by the client. 
    /// Based on that assumption, this class is much faster than its alternatives. Constrained dofs are ignored.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    public class UniformDofOrderingStrategy : IFreeDofOrderingStrategy
    {
        private readonly IReadOnlyList<IDofType> dofsPerNode;

        public UniformDofOrderingStrategy(IReadOnlyList<IDofType> dofsPerNode)
        {
            this.dofsPerNode = dofsPerNode;
        }

        public (int numGlobalFreeDofs, DofTable globalFreeDofs) OrderGlobalDofs(IModel model)
            => OrderFreeDofsOfNodeSet(model.EnumerateNodes());


        public (int numSubdomainFreeDofs, DofTable subdomainFreeDofs) OrderSubdomainDofs(ISubdomain subdomain)
            => OrderFreeDofsOfNodeSet(subdomain.Nodes);


        private (int numFreeDofs, DofTable freeDofs) OrderFreeDofsOfNodeSet(IEnumerable<INode> sortedNodes)
        {
            var freeDofs = new DofTable();
            int dofCounter = 0;
            foreach (INode node in sortedNodes)
            {
				var constrainedDofs = new HashSet<IDofType>();
				foreach (Constraint constraint in node.Constraints)
				{
					constrainedDofs.Add(constraint.DOF);
				}

                foreach (IDofType dof in dofsPerNode)
                {
					if (!constrainedDofs.Contains(dof))
					{
						freeDofs[node, dof] = dofCounter++;
					}
                }
            }
            return (dofCounter, freeDofs);
        }
    }
}
