using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Cracks.FriesPropagation
{
	public class LocalizedStressField
	{
		private readonly IAlgebraicModel algebraicModel;
		private readonly int dimension;
		private readonly int tensorLength;
		private readonly IGlobalVector totalDisplacements;

		private readonly Dictionary<int, double[]> nodalStresses = new Dictionary<int, double[]>();
		//private readonly Dictionary<int, double[]> elementDisplacements = new Dictionary<int, double[]>();

		public LocalizedStressField(int dimension, IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements)
		{
			this.dimension = dimension;
			this.algebraicModel = algebraicModel;
			this.totalDisplacements = totalDisplacements;
			if (dimension == 2)
			{
				tensorLength = 3;
			}
			else
			{
				Debug.Assert(dimension == 3);
				tensorLength = 6;
			}
		}

		public IReadOnlyDictionary<int, double[]> StoredNodalStresses => nodalStresses;

		public double[] CalcStressesAtNode(XNode node)
		{
			bool areNodeStressesCached = nodalStresses.TryGetValue(node.ID, out double[] stresses);
			if (!areNodeStressesCached)
			{
				stresses = CalcSmoothedStresses(node);
				nodalStresses[node.ID] = stresses;
			}
			return stresses;
		}

		public double[] CalcStressesAtPoint(double[] pointNatural, IXFiniteElement element)
		{
			var stressesAtElementNodes = new List<double[]>();
			foreach (XNode node in element.Nodes)
			{
				stressesAtElementNodes.Add(CalcStressesAtNode(node));
			}
			return InterpolateNodalStresses(pointNatural, element, stressesAtElementNodes);
		}

		private static double[] InterpolateNodalStresses(double[] pointNatural, IXFiniteElement element,
			List<double[]> stressesAtElementNodes)
		{
			int tensorLength = stressesAtElementNodes[0].Length;
			var stressesAtPoint = new double[tensorLength];
			double[] shapeFunctions = element.Interpolation.EvaluateFunctionsAt(pointNatural);
			for (int n = 0; n < element.Nodes.Count; ++n)
			{
				double N = shapeFunctions[n];
				double[] stressesAtNode = stressesAtElementNodes[n];
				for (int i = 0; i < tensorLength; ++i)
				{
					stressesAtPoint[i] += N * stressesAtNode[i];
				}
			}
			return stressesAtPoint;
		}

		private double[] CalcSmoothedStresses(XNode node)
		{
			var stressesAvg = new double[tensorLength];
			int numElements = 0;
			foreach (IXCrackElement element in node.ElementsDictionary.Values)
			{
				++numElements;
				var stresses = CalcStressesOfElementAtNode(node, element);

				for (int i = 0; i < tensorLength; ++i)
				{
					stressesAvg[i] += stresses[i];
				}
			}

			for (int i = 0; i < tensorLength; ++i)
			{
				stressesAvg[i] /= numElements;
			}
			return stressesAvg;
		}

		private double[] CalcStressesOfElementAtNode(XNode node, IXCrackElement element)
		{
			int nodeIdx = element.Nodes.FindFirstIndex(node);
			double[] nodeCoordsNatural = element.Interpolation.NodalNaturalCoordinates[nodeIdx];
			double[] elementDisplacements = algebraicModel.ExtractElementVector(totalDisplacements, element);
			DirichletElementLoad.ApplyBoundaryConditions(element, elementDisplacements);
			(_, double[] stresses) = element.CalcStrainsStressesAt(nodeCoordsNatural, elementDisplacements);
			return stresses;
		}
	}
}
