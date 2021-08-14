using System;
using System.Collections.Generic;
using System.Text;
using MGroup.FEM.Entities;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Solvers.DofOrdering;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Tests.SpecialSolvers
{
	public static class ModelUtilities
	{
		public static void DecomposeIntoSubdomains(this XModel<IXCrackElement> model, int numSubdomains, 
			Func<int, int> getSubdomainOfElement)
		{
			model.Subdomains.Clear();
			foreach (XNode node in model.Nodes.Values) node.Subdomains.Clear();
			foreach (IXCrackElement element in model.Elements.Values) element.SetSubdomainID(int.MinValue);

			for (int s = 0; s < numSubdomains; ++s)
			{
				model.Subdomains[s] = new XSubdomain<IXCrackElement>(s);
			}
			foreach (IXCrackElement element in model.Elements.Values)
			{
				XSubdomain<IXCrackElement> subdomain = model.Subdomains[getSubdomainOfElement(element.ID)];
				subdomain.Elements.Add(element);
			}

			model.ConnectDataStructures();
		}
	}
}
