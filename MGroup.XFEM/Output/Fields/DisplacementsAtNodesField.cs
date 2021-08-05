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
    public class DisplacementsAtNodesField
    {
        private readonly XModel<IXMultiphaseElement> model;

        public DisplacementsAtNodesField(XModel<IXMultiphaseElement> model)
        {
            this.model = model;
        }

        public Dictionary<double[], double[]> CalcValuesAtVertices(IVectorView solution)
        {
            if (model.Subdomains.Count != 1) throw new NotImplementedException();
            XSubdomain subdomain = model.Subdomains.First().Value;
            DofTable dofTable = subdomain.FreeDofOrdering.FreeDofs;

            IDofType[] dofsPerNode;
            if (model.Dimension == 2)
            {
                dofsPerNode = new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY };
            }
            else
            {
                dofsPerNode = new IDofType[] 
                { 
                    StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ 
                };
            }

            var result = new Dictionary<double[], double[]>();
            foreach (XNode node in model.XNodes)
            {
                double[] displacementsOfNode = new double[dofsPerNode.Length];
                for (int i = 0; i < dofsPerNode.Length; ++i)
                {
                    bool isFreeDof = dofTable.TryGetValue(node, dofsPerNode[i], out int index);
                    if (isFreeDof) displacementsOfNode[i] = solution[index];
                    else
                    {
                        Constraint constraint = node.Constraints.Find(c => c.DOF == dofsPerNode[i]);
                        displacementsOfNode[i] = constraint.Amount;
                    }
                }
                result[node.Coordinates] = displacementsOfNode;
            }
            return result;
        }
    }
}
