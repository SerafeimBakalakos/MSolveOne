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
	public class SmoothStrainStressField_v2
	{
		private readonly int dimension;
		private readonly IXModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ConformingContinuousMesh outMesh;

		public SmoothStrainStressField_v2(IXModel model, IAlgebraicModel algebraicModel, ConformingContinuousMesh outMesh)
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
			var elementVectors = new Dictionary<int, double[]>();

			foreach (int v in outMesh.Vertices.Keys)
			{
				// Keys are the element IDs
				var strainsAtPoint = new Dictionary<int, double[]>();
				var stressesAtPoint = new Dictionary<int, double[]>();

				foreach (var pair in outMesh.VerticesCoordinates[v].NaturalCoords)
				{
					var element = (IXCrackElement)(pair.Key);
					double[] naturalCoords = pair.Value;

					// Isolate element displacements (std, enr)
					bool isElementProcessed = elementVectors.TryGetValue(element.ID, out double[] elementVector);
					if (!isElementProcessed)
					{
						elementVector = algebraicModel.ExtractElementVector(solution, element);
						DirichletElementLoad.ApplyBoundaryConditions(element, elementVector);
					}

					(double[] strains, double[] stresses) = element.CalcStrainsStressesAt(naturalCoords, elementVector);
					strainsAtPoint[element.ID] = strains;
					stressesAtPoint[element.ID] = stresses;
				}
				outStrainTensors[v] = AverageTensor(strainsAtPoint);
				outStressTensors[v] = AverageTensor(stressesAtPoint);
			}
			return (outStrainTensors, outStressTensors);
		}

		private double[] AverageTensor(Dictionary<int, double[]> tensorPerElement)
		{
			int numElements = tensorPerElement.Count;
			int tensorLength = (model.Dimension == 2) ? 3 : 6;
			var result = new double[tensorLength];
			foreach (double[] tensor in tensorPerElement.Values)
			{
				for (int i = 0; i < tensorLength; ++i)
				{
					result[i] += tensor[i] / numElements;
				}
			}
			return result;
		}
	}
}
