using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.Environments.Mpi;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;
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

		public static IComputeEnvironment CreateEnvironment(this EnvironmentChoice environmentChoice)
		{
			if (environmentChoice == EnvironmentChoice.SequentialSharedEnvironment) return new SequentialSharedEnvironment();
			else if (environmentChoice == EnvironmentChoice.TplSharedEnvironment) return new TplSharedEnvironment();
			else if (environmentChoice == EnvironmentChoice.MklEnvironment) return new MpiEnvironment();
			else throw new NotImplementedException();
		}

		public static Table<int, int, double> FindNodalFieldValues(ISubdomain subdomain, ISubdomainFreeDofOrdering freeDofs,
			IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			var result = new Table<int, int, double>();

			// Free dofs
			foreach ((INode node, IDofType dof, int freeDofIdx) in freeDofs.FreeDofs)
			{
				result[node.ID, AllDofs.GetIdOfDof(dof)] = algebraicModel.ExtractSingleValue(solution, node, dof);
			}

			// Constrained dofs
			foreach (INode node in subdomain.Nodes)
			{
				foreach (Constraint dirichlet in node.Constraints)
				{
					result[node.ID, AllDofs.GetIdOfDof(dirichlet.DOF)] = dirichlet.Amount;
				}
			}

			return result;
		}
	}
}
