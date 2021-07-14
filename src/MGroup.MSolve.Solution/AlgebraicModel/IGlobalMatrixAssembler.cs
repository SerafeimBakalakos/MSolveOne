using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution.LinearSystem;

namespace MGroup.MSolve.Solution.AlgebraicModel
{
    /// <summary>
    /// Builds the matrix of the linear system that will be solved.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    public interface IGlobalMatrixAssembler
    {
		/// <summary>
		/// Builds the linear system matrix that corresponds to the free (unconstrained) freedom degrees.
		/// </summary>
		/// <param name="elementMatrixProvider">
		/// Determines the matrix calculated for each element (e.g. stiffness, mass, etc.)
		/// </param>
		IGlobalMatrix BuildGlobalMatrix(Func<int, IEnumerable<IElement>> accessElements, 
			IElementMatrixProvider elementMatrixProvider);

		/// <summary>
		/// Rebuilds the minimum necessary portions of the linear system matrix, depending on if 
		/// <see cref="IElementMatrixPredicate.MustBuildMatrixForElement(IElement)"/> of <paramref name="predicate"/>
		/// is satisfied for one or more elements. Even if <paramref name="predicate"/> is not satisfied for an element,
		/// its matrix may still be rebuilt, due to the behavior of neighboring elements. 
		/// For all elements that had their matrix rebuilt, 
		/// <see cref="IElementMatrixPredicate.ProcessElementAfterBuildingMatrix(IElement)"/>
		/// will be called afterwards. 
		/// For all elements that did not have their matrix rebuilt, 
		/// <see cref="IElementMatrixPredicate.ProcessElementAfterNotBuildingMatrix(IElement)"/> will be called 
		/// afterwards. 
		/// </summary>
		/// <param name="accessElements"></param>
		/// <param name="elementMatrixProvider"></param>
		/// <param name="predicate"></param>
		void RebuildGlobalMatrixPartially(IGlobalMatrix currentMatrix, Func<int, IEnumerable<IElement>> accessElements,
			IElementMatrixProvider elementMatrixProvider, IElementMatrixPredicate predicate);
	}
}
