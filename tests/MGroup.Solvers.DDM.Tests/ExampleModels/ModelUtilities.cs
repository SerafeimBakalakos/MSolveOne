using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;

namespace MGroup.Solvers.DDM.Tests.ExampleModels
{
	public static class ModelUtilities
	{
		public static void OrderDofs(IModel model)
		{
			throw new NotImplementedException();
			//var dofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
			//foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			//{
			//    subdomain.FreeDofOrdering = dofOrderer.OrderFreeDofs(subdomain);
			//}
		}
	}
}
