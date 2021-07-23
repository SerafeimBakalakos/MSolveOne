using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.Results
{
	public class NodalResults
	{
		public NodalResults(Table<int, int, double> data)
		{
			Data = data;
		}

		public Table<int, int, double> Data { get; }

		/// <summary>
		/// ||computedNodalValues - expectedNodalValues|| / ||expectedNodalValues||
		/// </summary>
		/// <param name="expectedNodalValues"></param>
		/// <param name="computedNodalValues"></param>
		/// <returns></returns>
		public static double CalcDeviationNorm(Table<int, int, double> expectedNodalValues,
			Table<int, int, double> computedNodalValues)
		{
			//TODO: Table should define DoEntrywise() and reduction methods.
			Debug.Assert(expectedNodalValues.EntryCount == computedNodalValues.EntryCount);
			var diff = new Table<int, int, double>();
			double sumNumerator = 0.0;
			double sumDenominator = 0.0;
			foreach ((int node, int dof, double computedValue) in computedNodalValues)
			{
				bool expectedValueExists = expectedNodalValues.TryGetValue(node, dof, out double expectedValue);
				Debug.Assert(expectedValueExists, $"Node {node} dof {dof}: No expected value provided");

				sumNumerator += (computedValue - expectedValue) * (computedValue - expectedValue);
				sumDenominator += expectedValue * expectedValue;
			}
			return Math.Sqrt(sumNumerator) / Math.Sqrt(sumDenominator);
		}

		public bool IsSuperSetOf(NodalResults other, double tolerance, out string msg)
		{
			var comparer = new ValueComparer(tolerance);
			foreach ((int node, int dof, double computedValue) in other.Data)
			{
				bool expectedValueExists = this.Data.TryGetValue(node, dof, out double expectedValue);
				if (!expectedValueExists)
				{
					msg = $"Node {node} dof {dof}: No expected value provided";
					return false;
				}

				if (!comparer.AreEqual(expectedValue, computedValue))
				{
					msg = $"Node {node} dof {dof}: expected = {expectedValue}, computed = {computedValue}";
					return false;
				}
			}

			msg = string.Empty;
			return true;
		}
	}
}
