using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.Assemblers
{
	public class SubdomainVectorAssembler
	{
		public void AddToSubdomainVector(IEnumerable<IElement> elements, Vector subdomainVector,
			IElementVectorProvider vectorProvider, ISubdomainFreeDofOrdering dofOrdering)
		{
			foreach (IElement element in elements)
			{
				(int[] elementDofs, int[] subdomainDofs) = dofOrdering.MapFreeDofsElementToSubdomain(element);
				var elementVector = Vector.CreateFromArray(vectorProvider.CalcVector(element));
				subdomainVector.AddIntoThisNonContiguouslyFrom(subdomainDofs, elementVector, elementDofs);
			}
		}
		
		public void AddToSubdomainVector(IEnumerable<IElementLoad> loads, Vector subdomainVector,
			ISubdomainFreeDofOrdering dofOrdering)
		{
			foreach (IElementLoad load in loads)
			{
				(int[] elementDofs, int[] subdomainDofs) = dofOrdering.MapFreeDofsElementToSubdomain(load.Element);
				var elementVector = Vector.CreateFromArray(load.CalcContribution());
				subdomainVector.AddIntoThisNonContiguouslyFrom(subdomainDofs, elementVector, elementDofs);
			}
		}

		public void AddToSubdomainVector(IEnumerable<INodalLoad> loads, Vector subdomainVector,
			ISubdomainFreeDofOrdering dofOrdering)
		{
			foreach (INodalLoad load in loads)
			{
				int dofIdx = dofOrdering.FreeDofs[load.Node, load.DOF];
				subdomainVector[dofIdx] += load.Amount;
			}
		}

		public void AddToSubdomainVector(IEnumerable<IAllNodeLoad> loads, Vector subdomainVector,
			ISubdomainFreeDofOrdering dofOrdering)
		{
			foreach (INode node in dofOrdering.FreeDofs.GetRows())
			{
				foreach (var dofIdxPair in dofOrdering.FreeDofs.GetDataOfRow(node))
				{
					IDofType dof = dofIdxPair.Key;
					int idx = dofIdxPair.Value;

					foreach (IAllNodeLoad load in loads)
					{
						if (load.DOF == dof)
						{
							subdomainVector[idx] = load.Amount;
						}
					}
				}
			}
		}
	}
}
