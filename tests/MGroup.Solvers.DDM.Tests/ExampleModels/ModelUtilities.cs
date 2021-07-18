using System;
using System.Collections.Generic;
using System.Text;
using MGroup.FEM.Entities;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;

namespace MGroup.Solvers.DDM.Tests.ExampleModels
{
	public static class ModelUtilities
	{
		public static ISubdomainFreeDofOrdering OrderDofs(ISubdomain subdomain)
		{
			var dofOrderer = new NodeMajorDofOrderingStrategy();
			(int numSubdomainFreeDofs, DofTable subdomainFreeDofs) = dofOrderer.OrderSubdomainDofs(subdomain);
			var dofOrdering = new SubdomainFreeDofOrderingCaching(numSubdomainFreeDofs, subdomainFreeDofs);
			return dofOrdering;
		}

		public static void DecomposeIntoSubdomains(this Model model, int numSubdomains, Func<int, int> getSubdomainOfElement)
		{
			model.SubdomainsDictionary.Clear();
			foreach (Node node in model.NodesDictionary.Values) node.SubdomainsDictionary.Clear();
			foreach (Element element in model.ElementsDictionary.Values) element.Subdomain = null;

			for (int s = 0; s < numSubdomains; ++s)
			{
				model.SubdomainsDictionary[s] = new Subdomain(s);
			}
			foreach (Element element in model.ElementsDictionary.Values)
			{
				Subdomain subdomain = model.SubdomainsDictionary[getSubdomainOfElement(element.ID)];
				subdomain.Elements.Add(element);
			}

			model.ConnectDataStructures();
		}
	}
}
