using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Matrices.Builders;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;

namespace MGroup.XFEM.Solvers.PaisReanalysis
{
	public class ReanalysisWholeMatrixAssembler : ISubdomainMatrixAssembler<DokMatrixAdapter>
	{
		private const string name = "ReanalysisWholeMatrixAssembler"; // for error messages
		private readonly bool sortColsOfEachRow;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="sortColsOfEachRow">
		/// Sorting the columns of each row in the CSC storage format may increase performance of the factorization and 
		/// back/forward substitutions. It is recommended to set it to true.
		/// </param>
		public ReanalysisWholeMatrixAssembler(bool sortColsOfEachRow = true)
		{
			this.sortColsOfEachRow = sortColsOfEachRow;
		}

		public DokMatrixAdapter BuildGlobalMatrix(ISubdomainFreeDofOrdering dofOrdering, IEnumerable<IElement> elements,
			IElementMatrixProvider matrixProvider)
		{
			int numFreeDofs = dofOrdering.NumFreeDofs;
			var subdomainMatrix = DokSymmetric.CreateEmpty(numFreeDofs);

			foreach (IElement element in elements)
			{
				// TODO: perhaps that could be done and cached during the dof enumeration to avoid iterating over the dofs twice
				(int[] elementDofIndices, int[] subdomainDofIndices) = dofOrdering.MapFreeDofsElementToSubdomain(element);
				IMatrix elementMatrix = matrixProvider.Matrix(element);
				subdomainMatrix.AddSubmatrixSymmetric(elementMatrix, elementDofIndices, subdomainDofIndices);
			}

			// Treat inactive/removed enriched dofs.
			subdomainMatrix.SetStructuralZeroDiagonalEntriesToUnity();
			return new DokMatrixAdapter(subdomainMatrix);

			//(double[] values, int[] rowIndices, int[] colOffsets) = subdomainMatrix.BuildSymmetricCscArrays(sortColsOfEachRow);
			//return SymmetricCscMatrix.CreateFromArrays(numFreeDofs, values, cachedRowIndices, cachedColOffsets, false);
		}

		public ISubdomainMatrixAssembler<DokMatrixAdapter> Clone() => new ReanalysisWholeMatrixAssembler(sortColsOfEachRow);

		public void HandleDofOrderingWasModified()
		{
		}
	}
}
