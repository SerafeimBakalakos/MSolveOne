using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Output.Fields
{
    public class TemperatureAtNodesField
    {
        private readonly XModel<IXMultiphaseElement> model;

        public TemperatureAtNodesField(XModel<IXMultiphaseElement> model)
        {
            this.model = model;
        }

        public Dictionary<double[], double> CalcValuesAtVertices(IVectorView solution)
        {
            if (model.Subdomains.Count != 1) throw new NotImplementedException();
            XSubdomain subdomain = model.Subdomains.First().Value;
            DofTable dofTable = subdomain.FreeDofOrdering.FreeDofs;

            var result = new Dictionary<double[], double>();
            foreach (XNode node in model.XNodes)
            {
                bool isFreeDof = dofTable.TryGetValue(node, ThermalDof.Temperature, out int stdDof);
                if (isFreeDof) result[node.Coordinates] = solution[stdDof];
                else result[node.Coordinates] = node.Constraints[0].Amount;
            }
            return result;
        }
    }
}
