using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Reduction;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.IsoXFEM
{
    public class StructuralPerfomance
    {
        public Vector strainEnergy;
        public Vector strainEnergyDensity;
        public Vector nodalStrainEnergyDensity;
		private readonly Dictionary<int, XNode> nodes  = new Dictionary<int, XNode>();
		private readonly Dictionary<int, IsoXfemElement2D> elements  = new Dictionary<int, IsoXfemElement2D>();
		private readonly Vector displacements;
        private readonly double initialArea;
        public StructuralPerfomance(Dictionary<int, XNode> nodes, Dictionary<int, IsoXfemElement2D> elements, double initialArea, Vector displacements)
        {
			this.nodes = nodes;
			this.elements = elements;
            this.initialArea = initialArea;
            this.displacements = displacements;
        }
        public void ComputeStrainEnergyandStrainEnergyDensity()
        {
            strainEnergy = Vector.CreateZero(elements.Count);
            strainEnergyDensity = Vector.CreateZero(elements.Count);           
            for (int i = 0; i < elements.Count; i++)
            {
                var dofsofelement = elements[i].dofsOfElement;
                Vector Ue = displacements.GetSubvector(dofsofelement);
                Matrix transposeUe = Matrix.CreateZero(1, 8);
                for (int j = 0; j < 8; j++)
                {
                    transposeUe[0, j] = Ue[j];
                }
                Matrix firstmult = transposeUe.Scale(0.5);
                Matrix secmult = firstmult.MultiplyRight(elements[i].stiffnessOfElement);
                Vector result = secmult * Ue;
                strainEnergy[i] = result[0];
                strainEnergyDensity[i] = strainEnergy[i] / initialArea;
            }           
        }
        public void ComputeNodalStrainEnergyDensity()
        {
           nodalStrainEnergyDensity = Vector.CreateZero(nodes.Count);
			//foreach (IsoXfemElement2D element in model.elements)
			//{
			//	foreach (XNode node in element.Nodes) node.ElementsDictionary[element.ID] = element;
			//}	
			for (int i = 0; i < nodes.Count; i++)
			{
				var node = nodes[i];
				var strainEnergyOfTheseElements = 0.0;
				foreach (var item in node.ElementsDictionary.Keys)
				{
					strainEnergyOfTheseElements += strainEnergyDensity[node.ElementsDictionary[item].ID];
				}
				nodalStrainEnergyDensity[i] = strainEnergyOfTheseElements / node.ElementsDictionary.Count;
			}
			//for (int i = 0; i < model.nodes.Count; i++)
   //         {
   //             var node = model.nodes[i];
   //             var strainEnergyOfTheseElements = 0.0;
   //             for (int j = 0; j <node.ElementsDictionary.Count/*.elementsOnNode.Count*/ ; j++)
   //             {				
			//		strainEnergyOfTheseElements += strainEnergyDensity[node.ElementsDictionary[j].ID];                    
   //             }
   //             nodalStrainEnergyDensity[i] = strainEnergyOfTheseElements / node.ElementsDictionary.Count;                               
   //         }
        }
    }
}
