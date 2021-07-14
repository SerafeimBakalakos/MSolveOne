using System.Collections.Generic;
using MGroup.MSolve.Discretization;


namespace MGroup.Solvers.DofOrdering
{
    /// <summary>
    /// Orders the unconstrained freedom degrees (dofs) of the physical model, by assigning an index to each unique dof. These 
    /// indices are used in vectors and matrices that contain quantities for the whole model (or its subdomains) to locate the 
    /// contribution of each dof.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    public interface IDofOrderer
    {
        /// <summary>
        /// Finds an ordering for the unconstrained freedom degrees of the physical model and its subdomains.
        /// </summary>
        /// <param name="model">The physical model.</param>
        IGlobalFreeDofOrdering OrderFreeDofs(IModel model);
    }
}
