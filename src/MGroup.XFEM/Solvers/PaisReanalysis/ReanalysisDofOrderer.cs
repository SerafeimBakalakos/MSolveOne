using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.XFEM.Enrichment;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class ReanalysisDofOrderer : IDofOrderer
	{
		private readonly Predicate<INode> canNodeBeEnriched;

		public ReanalysisDofOrderer(Predicate<INode> canNodeBeEnriched)
		{
			this.canNodeBeEnriched = canNodeBeEnriched;
		}

		public ISubdomainFreeDofOrdering OrderFreeDofs(ISubdomain subdomain, ActiveDofs allDofs)
		{
			List<IDofType> enrichedDofs = FindEnrichedDofs(allDofs);
			Dictionary<int, List<IDofType>> nodalDOFTypesDictionary = GatherDofsAtElementNodes(subdomain);

			int numFreeDofs = 0;
			var freeDofs = new IntDofTable();
			foreach (INode node in subdomain.EnumerateNodes())
			{
				// Find unique dofs of this node
				List<IDofType> dofsOfNode = nodalDOFTypesDictionary[node.ID];
				if (canNodeBeEnriched(node))
				{
					dofsOfNode.AddRange(enrichedDofs);
				}
				dofsOfNode = dofsOfNode.Distinct().ToList();

				// Assign each dof a global index
				foreach (IDofType dofType in dofsOfNode)
				{
					bool isFreeDof = IsFreeDof(node, dofType);
					if (isFreeDof)
					{
						freeDofs[node.ID, allDofs.GetIdOfDof(dofType)] = numFreeDofs;
						numFreeDofs++;
					}
				}
			}

			// Reorder dofs
			var dofOrdering = new SubdomainFreeDofOrderingCaching(numFreeDofs, freeDofs, allDofs);
			var reorderingStrategy = AmdReordering.CreateWithSuiteSparseAmd();
			reorderingStrategy.ReorderDofs(subdomain, dofOrdering);

			return dofOrdering;
		}

		private List<IDofType> FindEnrichedDofs(ActiveDofs allDofs)
		{
			SortedDictionary<int, IDofType> sortedDofs = allDofs.SortDofs();
			var enrichedDofs = new List<IDofType>();
			foreach (IDofType dof in sortedDofs.Values)
			{
				if (dof is EnrichedDof)
				{
					enrichedDofs.Add(dof);
				}
			}
			return enrichedDofs;
		}

		private Dictionary<int, List<IDofType>> GatherDofsAtElementNodes(ISubdomain subdomain)
		{
			var nodalDOFTypesDictionary = new Dictionary<int, List<IDofType>>(); //TODO: use Set instead of List
			foreach (IElement element in subdomain.EnumerateElements())
			{
				IReadOnlyList<IReadOnlyList<IDofType>> elementDofs
					= element.ElementType.DofEnumerator.GetDofTypesForDofEnumeration(element);
				for (int i = 0; i < element.Nodes.Count; i++)
				{
					if (!nodalDOFTypesDictionary.ContainsKey(element.Nodes[i].ID))
					{
						nodalDOFTypesDictionary.Add(element.Nodes[i].ID, new List<IDofType>());
					}
					nodalDOFTypesDictionary[element.Nodes[i].ID].AddRange(elementDofs[i]);
				}
			}
			return nodalDOFTypesDictionary;
		}

		private bool IsFreeDof(INode node, IDofType dofType)
		{
			foreach (var constraint in node.Constraints)
			{
				if (constraint.DOF == dofType)
				{
					return false;
				}
			}
			return true;
		}
	}
}
