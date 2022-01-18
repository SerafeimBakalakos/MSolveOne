using System;
using System.Collections.Generic;
using System.Text;

using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Reduction;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.AlgebraicModel;
using MGroup.XFEM.Entities;
using MGroup.XFEM.IsoXFEM.IsoXfemElements;

namespace MGroup.XFEM.IsoXFEM
{
    public class StructuralPerfomance
    {
        public Vector strainEnergy;
        public Vector strainEnergyDensity;
        public Vector nodalStrainEnergyDensity;
		private readonly int dimension;
		private readonly Dictionary<int, XNode> nodes  = new Dictionary<int, XNode>();
		private readonly Dictionary<int, IIsoXfemElement> elements  = new Dictionary<int, IIsoXfemElement>();
        private readonly double initialSize;
		private readonly IAlgebraicModel algebraicModel;
        public StructuralPerfomance(int dimension, Dictionary<int, XNode> nodes, Dictionary<int, IIsoXfemElement> elements, double initialSize, IAlgebraicModel algebraicModel)
        {
			this.dimension = dimension;
			this.nodes = nodes;
			this.elements = elements;
            this.initialSize = initialSize;
			this.algebraicModel = algebraicModel;
        }
		public (Vector strainEnergy, Vector nodalStrainEnergyDensity) ComputeStrainEnergyAndNodalSEDensity(IGlobalVector displacements)
		{
			var (strainEnergy, strainEnergyDensity)=ComputeStrainEnergyandStrainEnergyDensity(displacements);
			var nodalStrainEnergyDensity=ComputeNodalStrainEnergyDensity(strainEnergyDensity);
			return (strainEnergy, nodalStrainEnergyDensity);
		}
        public (Vector strainEnergy, Vector strainEnergyDensity) ComputeStrainEnergyandStrainEnergyDensity(IGlobalVector displacements)
        {
			Vector strainEnergy = Vector.CreateZero(elements.Count);
			Vector strainEnergyDensity = Vector.CreateZero(elements.Count);           
            for (int i = 0; i < elements.Count; i++)
            {
				var Ue = algebraicModel.ExtractElementVector(displacements, elements[i]);
				Matrix transposeUe = Matrix.CreateZero(1, elements[i].Nodes.Count*dimension);
				Vector displacementsElementVector = Vector.CreateZero(elements[i].Nodes.Count * dimension);
                for (int j = 0; j < Ue.Length; j++)
                {
                    transposeUe[0, j] = Ue[j];
					displacementsElementVector[j]= Ue[j];
				}
                Matrix firstmult = transposeUe.Scale(0.5);
                Matrix secmult = firstmult.MultiplyRight(elements[i].StiffnessOfElement);
                Vector result = secmult * displacementsElementVector;
                strainEnergy[i] = result[0];
                strainEnergyDensity[i] = strainEnergy[i] / initialSize;
            }
			return (strainEnergy, strainEnergyDensity);
        }
        public Vector ComputeNodalStrainEnergyDensity(Vector strainEnergyDensity)
        {
			Vector nodalStrainEnergyDensity = Vector.CreateZero(nodes.Count);
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
			return nodalStrainEnergyDensity;
        }
    }
}
