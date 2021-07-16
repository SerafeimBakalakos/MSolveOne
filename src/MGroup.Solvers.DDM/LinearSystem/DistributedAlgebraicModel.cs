using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution.Exceptions;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.DDM.LinearSystem
{
	public class DistributedAlgebraicModel<TMatrix> : IAlgebraicModel
		where TMatrix : class, IMatrix
	{
		private readonly int numSubdomains;
		private readonly IEnumerable<ISubdomain> subdomains;
		private readonly IComputeEnvironment environment;
		private readonly IModel model;
		private readonly IDofOrderer dofOrderer;
		private readonly Dictionary<int, ISubdomainMatrixAssembler<TMatrix>> subdomainMatrixAssemblers;
		//private readonly IDdmSolver solver;
		private readonly SubdomainVectorAssembler subdomainVectorAssembler = new SubdomainVectorAssembler();

		public DistributedAlgebraicModel(IComputeEnvironment environment, IModel model, IDofOrderer dofOrderer,
			ISubdomainMatrixAssembler<TMatrix> subdomainMatrixAssembler/*, IDdmSolver solver*/)
		{
			this.environment = environment;
			this.model = model;
			this.dofOrderer = dofOrderer;
			//this.solver = solver;
			subdomains = model.EnumerateSubdomains();
			numSubdomains = model.NumSubdomains;

			this.subdomainMatrixAssemblers = new Dictionary<int, ISubdomainMatrixAssembler<TMatrix>>();
			foreach (ISubdomain subdomain in subdomains)
			{
				this.subdomainMatrixAssemblers[subdomain.ID] = subdomainMatrixAssembler.Clone();
			}

			this.LinearSystem = new DistributedLinearSystem<TMatrix>(CheckCompatibleVector, CheckCompatibleMatrix);
			this.SubdomainLinearSystems = new Dictionary<int, SubdomainLinearSystem<TMatrix>>();
			foreach (ISubdomain subdomain in subdomains)
			{
				this.SubdomainLinearSystems[subdomain.ID] = new SubdomainLinearSystem<TMatrix>(this, subdomain.ID);
			}

			this.SubdomainTopology = new SubdomainTopology(environment, model, s => DofOrdering.SubdomainDofOrderings[s]);
			this.SubdomainTopology.FindCommonNodesBetweenSubdomains(); //TODO: what about problems where the mesh is repartitioned in some iterations?

			Observers = new HashSet<IAlgebraicModelObserver>();
		}

		public IGlobalFreeDofOrdering DofOrdering { get; set; }

		internal Guid Format { get; private set; }

		IGlobalLinearSystem IAlgebraicModel.LinearSystem => LinearSystem;

		public DistributedLinearSystem<TMatrix> LinearSystem { get; }

		public HashSet<IAlgebraicModelObserver> Observers { get; }

		public Dictionary<int, SubdomainLinearSystem<TMatrix>> SubdomainLinearSystems { get; }

		public SubdomainTopology SubdomainTopology { get; }

		public void AddToGlobalVector(Func<int, IEnumerable<IElement>> accessElements, IGlobalVector vector,
			IElementVectorProvider vectorProvider)
		{
			DistributedVector distributedVector = CheckCompatibleVector(vector);
			foreach (ISubdomain subdomain in subdomains)
			{
				ISubdomainFreeDofOrdering subdomainDofs = DofOrdering.SubdomainDofOrderings[subdomain.ID];
				IEnumerable<IElement> elements = accessElements(subdomain.ID);
				var subdomainVector = distributedVector.LocalVectors[subdomain.ID];
				subdomainVectorAssembler.AddToSubdomainVector(elements, subdomainVector, vectorProvider, subdomainDofs);
			}

			// Element loads at the same boundary dof must be summed across subdomains. 
			// This way the resulting global vector is the same as the corresponding global vector without domain decomposition.
			distributedVector.SumOverlappingEntries();
		}

		public void AddToGlobalVector(Func<int, IEnumerable<IElementLoad>> accessLoads, IGlobalVector vector)
		{
			DistributedVector distributedVector = CheckCompatibleVector(vector);
			foreach (ISubdomain subdomain in subdomains)
			{
				ISubdomainFreeDofOrdering subdomainDofs = DofOrdering.SubdomainDofOrderings[subdomain.ID];
				IEnumerable<IElementLoad> loads = accessLoads(subdomain.ID);
				var subdomainVector = distributedVector.LocalVectors[subdomain.ID];
				subdomainVectorAssembler.AddToSubdomainVector(loads, subdomainVector, subdomainDofs);
			}

			// Element loads at the same boundary dof must be summed across subdomains. 
			// This way the resulting global vector is the same as the corresponding global vector without domain decomposition.
			distributedVector.SumOverlappingEntries();
		}

		public void AddToGlobalVector(Func<int, IEnumerable<INodalLoad>> accessLoads, IGlobalVector vector)
		{
			DistributedVector distributedVector = CheckCompatibleVector(vector);
			foreach (ISubdomain subdomain in subdomains)
			{
				ISubdomainFreeDofOrdering subdomainDofs = DofOrdering.SubdomainDofOrderings[subdomain.ID];
				IEnumerable<INodalLoad> loads = accessLoads(subdomain.ID);
				var subdomainVector = distributedVector.LocalVectors[subdomain.ID];
				subdomainVectorAssembler.AddToSubdomainVector(loads, subdomainVector, subdomainDofs);
				//throw new NotImplementedException("This should probably be done privately by the solver without affecting global vectors used by other components");
				//solver.DistributeNodalLoads(loads, subdomainVector, subdomainDofs);
			}
			// Nodal loads at the same boundary dof are the same across all relevant subdomains, so we do not need to sum overlapping entries
		}

		public void AddToGlobalVector(IEnumerable<IAllNodeLoad> loads, IGlobalVector vector)
		{
			DistributedVector distributedVector = CheckCompatibleVector(vector);
			foreach (ISubdomain subdomain in subdomains)
			{
				ISubdomainFreeDofOrdering subdomainDofs = DofOrdering.SubdomainDofOrderings[subdomain.ID];
				var subdomainVector = distributedVector.LocalVectors[subdomain.ID];
				subdomainVectorAssembler.AddToSubdomainVector(loads, subdomainVector, subdomainDofs);
				//throw new NotImplementedException("This should probably be done privately by the solver without affecting global vectors used by other components");
				//solver.DistributeAllNodalLoads(subdomainVector, subdomainDofs);
			}
			// Nodal loads at the same boundary dof are the same across all relevant subdomains, so we do not need to sum overlapping entries
		}

		public IGlobalMatrix BuildGlobalMatrix(Func<int, IEnumerable<IElement>> accessElements, IElementMatrixProvider elementMatrixProvider)
		{
			var globalMatrix = new DistributedMatrix<TMatrix>(Format, CheckCompatibleVector, CheckCompatibleMatrix);
			foreach (ISubdomain subdomain in subdomains)
			{
				ISubdomainFreeDofOrdering subdomainDofs = DofOrdering.SubdomainDofOrderings[subdomain.ID];
				TMatrix matrix = subdomainMatrixAssemblers[subdomain.ID].BuildGlobalMatrix(
						subdomainDofs, accessElements(subdomain.ID), elementMatrixProvider);
				globalMatrix.LocalMatrices[subdomain.ID] = matrix;
			}
			return globalMatrix;
		}

		IGlobalVector IGlobalVectorAssembler.CreateZeroVector() => CreateZeroVector();

		public DistributedVector CreateZeroVector()
		{
			var result = new DistributedVector(Format, CheckCompatibleVector, subdomains, DofOrdering);
			foreach (ISubdomain subdomain in subdomains)
			{
				int size = DofOrdering.SubdomainDofOrderings[subdomain.ID].NumFreeDofs;
				result.LocalVectors[subdomain.ID] = Vector.CreateZero(size);
			}
			return result;
		}

		public void DoPerElement(Func<int, IEnumerable<IElement>> accessElements, Action<IElement> elementAction)
		{
			foreach (ISubdomain subdomain in subdomains)
			{
				foreach (IElement element in accessElements(subdomain.ID))
				{
					elementAction(element);
				}
			}
		}

		public double[] ExtractElementVector(IGlobalVector vector, IElement element)
		{
			DistributedVector distributedVector = CheckCompatibleVector(vector);
			int s = element.Subdomain.ID;
			ISubdomainFreeDofOrdering subdomainDofs = DofOrdering.SubdomainDofOrderings[s];
			return subdomainDofs.ExtractVectorElementFromSubdomain(element, distributedVector.LocalVectors[s]);
		}

		public double ExtractSingleValue(IGlobalVector vector, INode node, IDofType dof) //TODO: Dedicated classes to extract values.
		{
			DistributedVector distributedVector = CheckCompatibleVector(vector);
			if (node.SubdomainsDictionary.Count == 1) // Internal nodes are straightforward
			{
				ISubdomain subdomain = node.SubdomainsDictionary.First().Value;
				ISubdomainFreeDofOrdering subdomainDofs = DofOrdering.SubdomainDofOrderings[subdomain.ID];
				bool dofExists = subdomainDofs.FreeDofs.TryGetValue(node, dof, out int dofIdx);
				if (dofExists)
				{
					return distributedVector.LocalVectors[subdomain.ID][dofIdx];
				}
				else
				{
					throw new KeyNotFoundException("The requested (node, dof) is not included in the provided vector.");
				}
			}
			else // Boundary nodes are tricky
			{
				double tol = 1E-6; // TODO: this should be a parameter of the dedicated object that extracts values
				var comparer = new ValueComparer(tol);
				var values = new List<double>();
				foreach (ISubdomain subdomain in node.SubdomainsDictionary.Values)
				{
					ISubdomainFreeDofOrdering subdomainDofs = DofOrdering.SubdomainDofOrderings[subdomain.ID];
					bool dofExists = subdomainDofs.FreeDofs.TryGetValue(node, dof, out int dofIdx);
					if (dofExists)
					{
						// It is possible that this dof is activated by the elements of only 1 subdomain
						values.Add(distributedVector.LocalVectors[subdomain.ID][dofIdx]);
					}
				}

				if (values.Count == 0)
				{
					// It is also possible that the client is stupid
					throw new KeyNotFoundException("The requested (node, dof) is not included in the provided vector.");
				}
				else
				{
					double sum = values[0];
					for (int i = 1; i < values.Count; ++i)
					{
						if (comparer.AreEqual(values[i - 1], values[i]))
						{
							sum += values[i];
						}
						else
						{
							throw new NotImplementedException("There are multiple different values of for this (node, dof) pair");
						}
					}
					return sum / values.Count;
				}
			}
		}

		public void OrderDofs()
		{
			DofOrdering = dofOrderer.OrderFreeDofs(model);
			SubdomainTopology.FindCommonDofsBetweenSubdomains();
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

		public void RebuildGlobalMatrixPartially(IGlobalMatrix currentMatrix, Func<int, IEnumerable<IElement>> accessElements, IElementMatrixProvider elementMatrixProvider, IElementMatrixPredicate predicate)
		{
			DistributedMatrix<TMatrix> globalMatrix = CheckCompatibleMatrix(currentMatrix);
			foreach (ISubdomain subdomain in subdomains)
			{
				IEnumerable<IElement> subdomainElements = accessElements(subdomain.ID);
				TMatrix subdomainMatrix = subdomainMatrixAssemblers[subdomain.ID].RebuildSubdomainMatrix(
					subdomainElements, DofOrdering.SubdomainDofOrderings[subdomain.ID], elementMatrixProvider, predicate);
				if (subdomainMatrix != null)
				{
					//TODO: This is a good point to notify solvers, etc, if the processed matrix is the linear system matrix 
					globalMatrix.LocalMatrices[subdomain.ID] = subdomainMatrix;
				}
			}
		}

		internal DistributedMatrix<TMatrix> CheckCompatibleMatrix(IGlobalMatrix matrix)
		{
			// Casting inside here is usually safe since all global matrices should be created by this object
			if (matrix is DistributedMatrix<TMatrix> distributedMatrix)
			{
				if (distributedMatrix.Format == this.Format)
				{
					if (distributedMatrix.LocalMatrices.Count == numSubdomains)
					{
						return distributedMatrix;
					}
				}
			}
			throw new InvalidLinearSystemFormat("The provided matrix has a different format than the current linear system."
				+ $" Make sure it was created by the linear system with format = {Format},"
				+ $" corresponds to {numSubdomains} subdomains and that the type {typeof(TMatrix)} is used.");
		}

		internal DistributedVector CheckCompatibleVector(IGlobalVector vector)
		{
			// Casting inside here is usually safe since all global vectors should be created by the this object
			if (vector is DistributedVector distributedVector)
			{
				if (distributedVector.Format == this.Format)
				{
					if (distributedVector.LocalVectors.Count == numSubdomains)
					{
						return distributedVector;
					}
				}
			}
			throw new InvalidLinearSystemFormat("The provided vector has a different format than the current linear system."
				+ $" Make sure it was created by the linear system with format = {Format}"
				+ $" and corresponds to {numSubdomains} subdomains.");
		}
	}
}
