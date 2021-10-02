using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Mesh;

namespace MGroup.XFEM.Output.Fields
{
	public class SmoothStrainStressField
	{
		private const double offsetTol = 1E-6;

		private readonly int dimension;
		private readonly IXModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ContinuousOutputMesh outMesh;

		public SmoothStrainStressField(IXModel model, IAlgebraicModel algebraicModel, ContinuousOutputMesh outMesh)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.dimension = model.Dimension;
			this.outMesh = outMesh;
		}

		public (Dictionary<int, double[]> strains, Dictionary<int, double[]> stresses) CalcTensorsAtVertices(IGlobalVector solution)
		{
			if (model.NumSubdomains != 1) throw new NotImplementedException();

			var outStrainTensors = new Dictionary<int, double[]>();
			var outStressTensors = new Dictionary<int, double[]>();
			int tensorLength = (model.Dimension == 2) ? 3 : 6;
			foreach (int nodeID in model.Nodes.Keys)
			{
				outStrainTensors[nodeID] = new double[tensorLength];
				outStressTensors[nodeID] = new double[tensorLength];
			}

			foreach (IXCrackElement element in model.EnumerateElements())
			{
				double[] elementVector = algebraicModel.ExtractElementVector(solution, element);
				DirichletElementLoad.ApplyBoundaryConditions(element, elementVector);

				for (int n = 0; n < element.Nodes.Count; ++n)
				{
					int nodeID = element.Nodes[n].ID;
					double[] nodeNatural = element.Interpolation.NodalNaturalCoordinates[n];
					(double[] strains, double[] stresses) = element.CalcStrainsStressesAt(nodeNatural, elementVector);
					AddToTensor(outStrainTensors[nodeID], strains);
					AddToTensor(outStressTensors[nodeID], stresses);
				}
			}

			foreach (XNode node in model.Nodes.Values)
			{
				double coeff = 1.0 / node.ElementsDictionary.Count;
				ScaleTensor(outStrainTensors[node.ID], coeff);
				ScaleTensor(outStressTensors[node.ID], coeff);
			}
			return (outStrainTensors, outStressTensors);
		}

		private static void AddToTensor(double[] tensor, double[] other)
		{
			for (int i = 0; i < tensor.Length; ++i)
			{
				tensor[i] += other[i];
			}
		}

		private static void ScaleTensor(double[] tensor, double scalar)
		{
			for (int i = 0; i < tensor.Length; ++i)
			{
				tensor[i] *= scalar;
			}
		}
	}
}
