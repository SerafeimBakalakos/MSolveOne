using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.MSolve.Discretization.Loads
{
	public class DirichletElementLoad : IElementLoad
	{
		public DirichletElementLoad(IElement element)
		{
			this.Element = element;
		}

		public IElement Element { get; }

		/// <summary>
		/// If this returns true, then <see cref="CalcContribution"/> will return a zero vector.
		/// </summary>
		public bool IsZero()
		{
			foreach (INode node in Element.Nodes)
			{
				foreach (Constraint constraint in node.Constraints)
				{
					if (constraint.Amount != 0.0)
					{
						return false;
					}
				}
			}
			return true;
		}

		public double[] CalcContribution()
		{
			IMatrix elementK = Element.ElementType.StiffnessMatrix(Element);
			var elementNodalDisplacements = new double[elementK.NumColumns];
			ApplyBoundaryConditions(Element, elementNodalDisplacements);
			double[] localdSolution = elementNodalDisplacements;
			var elementEquivalentForces = elementK.Multiply(localdSolution);
			return elementEquivalentForces;
		}

		public static void ApplyBoundaryConditions(IElement element, double[] elementVector)
		{
			//TODO: It would be more convenient to have a DofTable per element, instead of IReadOnlyList<IReadOnlyList<IDofType>>

			IReadOnlyList<INode> nodes = element.ElementType.DofEnumerator.GetNodesForMatrixAssembly(element);
			IReadOnlyList<IReadOnlyList<IDofType>> dofs = element.ElementType.DofEnumerator.GetDofTypesForMatrixAssembly(element);
			int dofIdxOffset = 0;
			for (int n = 0; n < nodes.Count; ++n)
			{
				INode node = nodes[n];
				if (node.Constraints.Count > 0)
				{
					// Associate dofs with their indices into the element vector
					var dofIndices = new Dictionary<IDofType, int>();
					for (int d = 0; d < dofs[n].Count; ++d)
					{
						dofIndices[dofs[n][d]] = dofIdxOffset + d;
					}

					// Add the contributions from prescribed dirichlet conditions
					foreach (Constraint constraint in node.Constraints)
					{
						int idx = dofIndices[constraint.DOF];
						elementVector[idx] += constraint.Amount;
					}
				}

				dofIdxOffset += dofs[n].Count;
			}
		}
	}
}
