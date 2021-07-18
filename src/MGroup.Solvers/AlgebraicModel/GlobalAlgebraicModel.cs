using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Distributed.LinearAlgebraExtensions;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution.AlgebraicModel;

using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.LinearSystem;

namespace MGroup.Solvers.AlgebraicModel
{
	public class GlobalAlgebraicModel<TMatrix> : IAlgebraicModel
		where TMatrix : class, IMatrix
	{
		private readonly ISubdomain subdomain;
		private readonly IDofOrderer dofOrderer;
		private readonly ISubdomainMatrixAssembler<TMatrix> subdomainMatrixAssembler;
		private readonly SubdomainVectorAssembler subdomainVectorAssembler = new SubdomainVectorAssembler();

		public GlobalAlgebraicModel(IModel model, IDofOrderer dofOrderer,
			ISubdomainMatrixAssembler<TMatrix> subdomainMatrixAssembler)
		{
			this.dofOrderer = dofOrderer;
			this.subdomainMatrixAssembler = subdomainMatrixAssembler;
			subdomain = model.EnumerateSubdomains().First();
			this.LinearSystem = new GlobalLinearSystem<TMatrix>(CheckCompatibleVector, CheckCompatibleMatrix);
			Observers = new HashSet<IAlgebraicModelObserver>();
		}

		public ISubdomainFreeDofOrdering SubdomainFreeDofOrdering { get; private set; }

		public Guid Format { get; private set; }

		IGlobalLinearSystem IAlgebraicModel.LinearSystem => LinearSystem;

		public GlobalLinearSystem<TMatrix> LinearSystem { get; }

		public HashSet<IAlgebraicModelObserver> Observers { get; }

		public int SubdomainID { get; }

		public void AddToGlobalVector(Func<int, IEnumerable<IElement>> accessElements, IGlobalVector vector,
			IElementVectorProvider vectorProvider)
		{
			GlobalVector globalVector = CheckCompatibleVector(vector);
			ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrdering;
			IEnumerable<IElement> elements = accessElements(subdomain.ID);
			subdomainVectorAssembler.AddToSubdomainVector(elements, globalVector.SingleVector, vectorProvider, subdomainDofs);
		}

		public void AddToGlobalVector(Func<int, IEnumerable<INodalLoad>> accessLoads, IGlobalVector vector)
		{
			GlobalVector globalVector = CheckCompatibleVector(vector);
			ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrdering;
			IEnumerable<INodalLoad> loads = accessLoads(subdomain.ID);
			subdomainVectorAssembler.AddToSubdomainVector(loads, globalVector.SingleVector, subdomainDofs);
		}

		public void AddToGlobalVector(Func<int, IEnumerable<IElementLoad>> accessLoads, IGlobalVector vector)
		{
			GlobalVector globalVector = CheckCompatibleVector(vector);
			ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrdering;
			IEnumerable<IElementLoad> loads = accessLoads(subdomain.ID);
			subdomainVectorAssembler.AddToSubdomainVector(loads, globalVector.SingleVector, subdomainDofs);
		}

		public void AddToGlobalVector(IEnumerable<IAllNodeLoad> loads, IGlobalVector vector)
		{
			GlobalVector globalVector = CheckCompatibleVector(vector);
			ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrdering;
			subdomainVectorAssembler.AddToSubdomainVector(loads, globalVector.SingleVector, subdomainDofs);
		}

		public IGlobalMatrix BuildGlobalMatrix(
			Func<int, IEnumerable<IElement>> accessElements, IElementMatrixProvider elementMatrixProvider)
		{
			ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrdering;
			var globalMatrix = new GlobalMatrix<TMatrix>(Format, CheckCompatibleVector, CheckCompatibleMatrix);
			globalMatrix.SingleMatrix = subdomainMatrixAssembler.BuildGlobalMatrix(
				subdomainDofs, accessElements(subdomain.ID), elementMatrixProvider);
			return globalMatrix;
		}

		IGlobalVector IGlobalVectorAssembler.CreateZeroVector() => CreateZeroVector();

		//TODO: Should there also be a CreateZeroMatrix()?
		public GlobalVector CreateZeroVector()
		{
			var result = new GlobalVector(Format, CheckCompatibleVector);
			result.SingleVector = Vector.CreateZero(SubdomainFreeDofOrdering.NumFreeDofs);
			return result;
		}

		public void DoPerElement(Func<int, IEnumerable<IElement>> accessElements, Action<IElement> elementAction)
		{
			foreach (IElement element in accessElements(subdomain.ID))
			{
				elementAction(element);
			}
		}

		public double[] ExtractElementVector(IGlobalVector vector, IElement element)
		{
			GlobalVector globalVector = CheckCompatibleVector(vector);
			ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrdering;
			return subdomainDofs.ExtractVectorElementFromSubdomain(element, globalVector.SingleVector);
		}

		public double ExtractSingleValue(IGlobalVector vector, INode node, IDofType dof)
		{
			GlobalVector globalVector = CheckCompatibleVector(vector);
			ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrdering;
			bool dofExists = subdomainDofs.FreeDofs.TryGetValue(node, dof, out int dofIdx);
			if (dofExists)
			{
				return globalVector.SingleVector[dofIdx];
			}
			else
			{
				throw new KeyNotFoundException("The requested (node, dof) is not included in the provided vector.");
			}
		}

		public void OrderDofs()
		{
			SubdomainFreeDofOrdering = dofOrderer.OrderFreeDofs(subdomain);
			foreach (IAlgebraicModelObserver observer in Observers)
			{
				observer.HandleDofOrderWasModified();
			}

			// Define new format and recreate objects using it 
			Format = Guid.NewGuid();
			LinearSystem.Matrix = null;
			LinearSystem.RhsVector = CreateZeroVector();
			LinearSystem.Solution = CreateZeroVector();
		}

		public void RebuildGlobalMatrixPartially(
			IGlobalMatrix currentMatrix, Func<int, IEnumerable<IElement>> accessElements,
			IElementMatrixProvider elementMatrixProvider, IElementMatrixPredicate predicate)
		{
			GlobalMatrix<TMatrix> globalMatrix = CheckCompatibleMatrix(currentMatrix);

			var watch = new Stopwatch();
			watch.Start();

			IEnumerable<IElement> subdomainElements = accessElements(subdomain.ID);
			TMatrix subdomainMatrix = subdomainMatrixAssembler.RebuildSubdomainMatrix(
				subdomainElements, SubdomainFreeDofOrdering, elementMatrixProvider, predicate);
			if (subdomainMatrix != null)
			{
				//TODO: This is a good point to notify solvers, etc, if the processed matrix is the linear system matrix 
				globalMatrix.SingleMatrix = subdomainMatrix;
			}
		}

		internal GlobalMatrix<TMatrix> CheckCompatibleMatrix(IGlobalMatrix matrix)
		{
			// Casting inside here is usually safe since all global matrices should be created by this object
			if (matrix is GlobalMatrix<TMatrix> globalMatrix)
			{
				if (globalMatrix.Format == this.Format)
				{
					return globalMatrix;
				}
			}
			throw new NonMatchingFormatException("The provided matrix has a different format than the current linear system."
				+ $" Make sure it was created by the linear system with format = {Format}"
				+ $" and that the type {typeof(TMatrix)} is used.");
		}

		internal GlobalVector CheckCompatibleVector(IGlobalVector vector)
		{
			// Casting inside here is usually safe since all global vectors should be created by the this object
			if (vector is GlobalVector globalVector)
			{
				if (globalVector.Format == this.Format)
				{
					return globalVector;
				}
			}
			throw new NonMatchingFormatException("The provided vector has a different format than the current linear system."
				+ $" Make sure it was created by the linear system with format = {Format}.");
		}
	}
}
