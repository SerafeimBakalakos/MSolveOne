using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.DofOrdering;
using Xunit;

namespace MGroup.Solvers.DDM.Prototypes.Tests
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
		public static void AssertEqual(Table<int, int, double> expectedNodalValues, 
			Table<int, int, double> computedNodalValues, double tolerance)
		{
			var comparer = new ValueComparer(tolerance);
			foreach ((int node, int dof, double computedValue) in computedNodalValues)
			{
				bool expectedValueExists = expectedNodalValues.TryGetValue(node, dof, out double expectedValue);
				Assert.True(expectedValueExists, $"Node {node} dof {dof}: No expected value provided");
				Assert.True(comparer.AreEqual(expectedValue, computedValue),
					$"Node {node} dof {dof}: expected = {expectedValue}, computed = {computedValue}");
			}
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
