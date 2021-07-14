using System;
using System.Collections.Generic;
using System.Text;
using MGroup.FEM.Entities;

namespace MGroup.Solvers.DDM.Prototypes.Tests.ExampleModels
{
    public static class ModelUtilities
    {
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
