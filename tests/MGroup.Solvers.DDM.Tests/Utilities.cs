using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Environments;
using MGroup.Environments.Mpi;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.DofOrdering;
using Xunit;

namespace MGroup.Solvers.DDM.Tests
{
	public static class Utilities
	{
		public static bool AreEqual(int[] expected, int[] computed)
		{
			if (expected.Length != computed.Length)
			{
				return false;
			}
			for (int i = 0; i < expected.Length; ++i)
			{
				if (expected[i] != computed[i])
						{
					return false;
				}
			}
			return true;
		}

		/// <summary>
		/// 
		/// </summary>
		/// <param name="expectedNodalValues"></param>
		/// <param name="computedNodalValues">
		/// These can be a subset of <paramref name="expectedNodalValues"/>. Only the common ones will be checked.
		/// </param>
		public static void AssertSubset(Table<int, int, double> expectedNodalValues, 
			Table<int, int, double> computedNodalValues, double tolerance)
		{
			//TODO: Table<TRow, TCol, double> should define Equals(double tol).
			var comparer = new ValueComparer(tolerance);
			foreach ((int node, int dof, double computedValue) in computedNodalValues)
			{
				bool expectedValueExists = expectedNodalValues.TryGetValue(node, dof, out double expectedValue);
				Assert.True(expectedValueExists, $"Node {node} dof {dof}: No expected value provided");
				Assert.True(comparer.AreEqual(expectedValue, computedValue),
					$"Node {node} dof {dof}: expected = {expectedValue}, computed = {computedValue}");
			}
		}

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

		public static IComputeEnvironment CreateEnvironment(this EnvironmentChoice environmentChoice)
		{
			if (environmentChoice == EnvironmentChoice.SequentialSharedEnvironment) return new SequentialSharedEnvironment();
			else if (environmentChoice == EnvironmentChoice.TplSharedEnvironment) return new TplSharedEnvironment();
			else if (environmentChoice == EnvironmentChoice.MklEnvironment) return new MpiEnvironment();
			else throw new NotImplementedException();
		}

		public static Table<int, int, double> FindNodalFieldValues(ISubdomain subdomain, ISubdomainFreeDofOrdering freeDofs,
			IModel model, IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			var result = new Table<int, int, double>();

			// Free dofs
			foreach ((INode node, IDofType dof, int freeDofIdx) in freeDofs.FreeDofs)
			{
				result[node.ID, model.AllDofs.GetIdOfDof(dof)] = algebraicModel.ExtractSingleValue(solution, node, dof);
			}

			// Constrained dofs
			foreach (INode node in subdomain.Nodes)
			{
				foreach (Constraint dirichlet in node.Constraints)
				{
					result[node.ID, model.AllDofs.GetIdOfDof(dirichlet.DOF)] = dirichlet.Amount;
				}
			}

			return result;
		}
	}
}
