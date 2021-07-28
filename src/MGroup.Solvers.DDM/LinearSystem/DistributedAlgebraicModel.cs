using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.Results;


//TODO: Move all Extract~() methods to a dedicated DistributedValueExtractor class.
namespace MGroup.Solvers.DDM.LinearSystem
{
	public class DistributedAlgebraicModel<TMatrix> : IAlgebraicModel
		where TMatrix : class, IMatrix
	{
		private readonly IComputeEnvironment environment;
		private readonly IModel model;
		private readonly IDofOrderer dofOrderer;
		private readonly Dictionary<int, ISubdomainMatrixAssembler<TMatrix>> subdomainMatrixAssemblers;
		private readonly SubdomainVectorAssembler subdomainVectorAssembler;

		public DistributedAlgebraicModel(IComputeEnvironment environment, IModel model, IDofOrderer dofOrderer,
			ISubdomainMatrixAssembler<TMatrix> subdomainMatrixAssembler)
		{
			this.environment = environment;
			this.model = model;
			this.dofOrderer = dofOrderer;

			this.subdomainMatrixAssemblers = environment.CreateDictionaryPerNode(subdomainID => subdomainMatrixAssembler.Clone());
			this.subdomainVectorAssembler = new SubdomainVectorAssembler(model.AllDofs);

			this.LinearSystem = new DistributedLinearSystem<TMatrix>(CheckCompatibleVector, CheckCompatibleMatrix);
			this.SubdomainLinearSystems = environment.CreateDictionaryPerNode(
				subdomainID => new SubdomainLinearSystem<TMatrix>(this, subdomainID));

			this.SubdomainTopology = new SubdomainTopology(environment, model, s => SubdomainFreeDofOrderings[s]);
			this.SubdomainTopology.FindCommonNodesBetweenSubdomains(); //TODO: what about problems where the mesh is repartitioned in some iterations?

			Observers = new HashSet<IAlgebraicModelObserver>();
		}

		public ConcurrentDictionary<int, ISubdomainFreeDofOrdering> SubdomainFreeDofOrderings { get; } 
			= new ConcurrentDictionary<int, ISubdomainFreeDofOrdering>();

		internal Guid Format { get; private set; }

		//TODOMPI: Perhaps this and subdomain free dof orderings must be managed by a dedicated component. That component could 
		//		also perform some of all these tasks done by SubdomainTopology.
		public DistributedOverlappingIndexer FreeDofIndexer { get; private set; } 

		IGlobalLinearSystem IAlgebraicModel.LinearSystem => LinearSystem;

		public DistributedLinearSystem<TMatrix> LinearSystem { get; }

		public HashSet<IAlgebraicModelObserver> Observers { get; }

		public Dictionary<int, SubdomainLinearSystem<TMatrix>> SubdomainLinearSystems { get; }

		public SubdomainTopology SubdomainTopology { get; }

		public void AddToGlobalVector(Func<int, IEnumerable<IElement>> accessElements, IGlobalVector vector,
			IElementVectorProvider vectorProvider)
		{
			DistributedOverlappingVector distributedVector = FreeDofIndexer.CheckCompatibleVector(vector);
			environment.DoPerNode(subdomainID =>
			{
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[subdomainID];
				IEnumerable<IElement> elements = accessElements(subdomainID);
				var subdomainVector = distributedVector.LocalVectors[subdomainID];
				subdomainVectorAssembler.AddToSubdomainVector(elements, subdomainVector, vectorProvider, subdomainDofs);
			});

			// Element loads at the same boundary dof must be summed across subdomains. 
			// This way the resulting global vector is the same as the corresponding global vector without domain decomposition.
			distributedVector.SumOverlappingEntries();
		}

		public void AddToGlobalVector(Func<int, IEnumerable<IElementLoad>> accessLoads, IGlobalVector vector)
		{
			DistributedOverlappingVector distributedVector = FreeDofIndexer.CheckCompatibleVector(vector);
			environment.DoPerNode(subdomainID =>
			{
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[subdomainID];
				IEnumerable<IElementLoad> loads = accessLoads(subdomainID);
				var subdomainVector = distributedVector.LocalVectors[subdomainID];
				subdomainVectorAssembler.AddToSubdomainVector(loads, subdomainVector, subdomainDofs);
			});

			// Element loads at the same boundary dof must be summed across subdomains. 
			// This way the resulting global vector is the same as the corresponding global vector without domain decomposition.
			distributedVector.SumOverlappingEntries();
		}

		public void AddToGlobalVector(Func<int, IEnumerable<INodalLoad>> accessLoads, IGlobalVector vector)
		{
			DistributedOverlappingVector distributedVector = FreeDofIndexer.CheckCompatibleVector(vector);
			environment.DoPerNode(subdomainID =>
			{
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[subdomainID];
				IEnumerable<INodalLoad> loads = accessLoads(subdomainID);
				var subdomainVector = distributedVector.LocalVectors[subdomainID];
				subdomainVectorAssembler.AddToSubdomainVector(loads, subdomainVector, subdomainDofs);
			});

			// Nodal loads at the same boundary dof are the same across all relevant subdomains, 
			// so we do not need to sum overlapping entries
		}

		public void AddToGlobalVector(IEnumerable<IAllNodeLoad> loads, IGlobalVector vector)
		{
			DistributedOverlappingVector distributedVector = FreeDofIndexer.CheckCompatibleVector(vector);
			environment.DoPerNode(subdomainID =>
			{
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[subdomainID];
				var subdomainVector = distributedVector.LocalVectors[subdomainID];
				subdomainVectorAssembler.AddToSubdomainVector(loads, subdomainVector, subdomainDofs);
			});

			// Nodal loads at the same boundary dof are the same across all relevant subdomains, 
			// so we do not need to sum overlapping entries
		}

		public IGlobalMatrix BuildGlobalMatrix(Func<int, IEnumerable<IElement>> accessElements, 
			IElementMatrixProvider elementMatrixProvider)
		{
			var globalMatrix = new DistributedOverlappingMatrix<TMatrix>(FreeDofIndexer);
			environment.DoPerNode(subdomainID =>
			{
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[subdomainID];
				TMatrix matrix = subdomainMatrixAssemblers[subdomainID].BuildGlobalMatrix(
						subdomainDofs, accessElements(subdomainID), elementMatrixProvider);
				globalMatrix.LocalMatrices[subdomainID] = matrix;
			});
			return globalMatrix;
		}

		IGlobalVector IGlobalVectorAssembler.CreateZeroVector() => CreateZeroVector();

		public DistributedOverlappingVector CreateZeroVector() => new DistributedOverlappingVector(FreeDofIndexer);

		public void DoPerElement(Func<int, IEnumerable<IElement>> accessElements, Action<IElement> elementAction)
		{
			environment.DoPerNode(subdomainID =>
			{
				foreach (IElement element in accessElements(subdomainID))
				{
					elementAction(element);
				}
			});
		}

		public NodalResults ExtractAllResults(int subdomainID, IGlobalVector vector)
		{
			var results = new Table<int, int, double>();

			// Free dofs
			DistributedOverlappingVector distributedVector = CheckCompatibleVector(vector);
			Vector subdomainVector = distributedVector.LocalVectors[subdomainID];
			ISubdomainFreeDofOrdering subdomainFreeDofs = SubdomainFreeDofOrderings[subdomainID];
			foreach ((int node, int dof, int freeDofIdx) in subdomainFreeDofs.FreeDofs)
			{
				results[node, dof] = subdomainVector[freeDofIdx];
			}

			// Constrained dofs
			ISubdomain subdomain = model.GetSubdomain(subdomainID);
			foreach (INode node in subdomain.Nodes)
			{
				foreach (Constraint dirichlet in node.Constraints)
				{
					results[node.ID, model.AllDofs.GetIdOfDof(dirichlet.DOF)] = dirichlet.Amount;
				}
			}

			return new NodalResults(results);
		}

		public double[] ExtractElementVector(IGlobalVector vector, IElement element)
		{
			if (!(environment is SequentialSharedEnvironment) && !(environment is TplSharedEnvironment))
			{
				throw new NotImplementedException("We need to locate the correct subdomain first");
			}
			DistributedOverlappingVector distributedVector = CheckCompatibleVector(vector);
			int s = element.SubdomainID;
			ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[s];
			return subdomainDofs.ExtractVectorElementFromSubdomain(element, distributedVector.LocalVectors[s]);
		}

		public NodalResults ExtractGlobalResults(IGlobalVector vector, double differentValueTolerance)
		{
			if (!(environment is SequentialSharedEnvironment) && !(environment is TplSharedEnvironment))
			{
				throw new NotImplementedException();
			}
			var globalResults = new NodalResults(new Table<int, int, double>());
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				NodalResults subdomainResults = ExtractAllResults(subdomain.ID, vector);
				globalResults.UnionWith(subdomainResults, differentValueTolerance);
			}
			return globalResults;
		}

		public double ExtractSingleValue(IGlobalVector vector, INode node, IDofType dof) //TODO: Dedicated classes to extract values.
		{
			if (!(environment is SequentialSharedEnvironment) && !(environment is TplSharedEnvironment))
			{
				throw new NotImplementedException("We need to locate the correct subdomain first");
			}
			DistributedOverlappingVector distributedVector = CheckCompatibleVector(vector);
			if (node.Subdomains.Count == 1) // Internal nodes are straightforward
			{
				int s = node.Subdomains.First();
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[s];
				int dofID = model.AllDofs.GetIdOfDof(dof);
				bool dofExists = subdomainDofs.FreeDofs.TryGetValue(node.ID, dofID, out int dofIdx);
				if (dofExists)
				{
					return distributedVector.LocalVectors[s][dofIdx];
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
				foreach (int s in node.Subdomains)
				{
					ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[s];
					int dofID = model.AllDofs.GetIdOfDof(dof);
					bool dofExists = subdomainDofs.FreeDofs.TryGetValue(node.ID, dofID, out int dofIdx);
					if (dofExists)
					{
						// It is possible that this dof is activated by the elements of only 1 subdomain
						values.Add(distributedVector.LocalVectors[s][dofIdx]);
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
			environment.DoPerNode(subdomainID =>
			{
				ISubdomain subdomain = model.GetSubdomain(subdomainID);
				SubdomainFreeDofOrderings[subdomainID] = dofOrderer.OrderFreeDofs(subdomain, model.AllDofs);
			});
			SubdomainTopology.FindCommonDofsBetweenSubdomains();
			FreeDofIndexer = SubdomainTopology.CreateDistributedVectorIndexer(s => SubdomainFreeDofOrderings[s].FreeDofs);
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

		public void RebuildGlobalMatrixPartially(IGlobalMatrix currentMatrix, Func<int, IEnumerable<IElement>> accessElements, 
			IElementMatrixProvider elementMatrixProvider, IElementMatrixPredicate predicate)
		{
			DistributedOverlappingMatrix<TMatrix> distributedMatrix = 
				FreeDofIndexer.CheckCompatibleMatrix<TMatrix>(currentMatrix);
			environment.DoPerNode(subdomainID =>
			{
				IEnumerable<IElement> subdomainElements = accessElements(subdomainID);
				TMatrix subdomainMatrix = subdomainMatrixAssemblers[subdomainID].RebuildSubdomainMatrix(
					subdomainElements, SubdomainFreeDofOrderings[subdomainID], elementMatrixProvider, predicate);
				if (subdomainMatrix != null)
				{
					//TODO: This is a good point to notify solvers, etc, if the processed matrix is the linear system matrix 
					distributedMatrix.LocalMatrices[subdomainID] = subdomainMatrix;
				}
			});
		}

		internal DistributedOverlappingMatrix<TMatrix> CheckCompatibleMatrix(IGlobalMatrix matrix) 
			=> FreeDofIndexer.CheckCompatibleMatrix<TMatrix>(matrix);

		internal DistributedOverlappingVector CheckCompatibleVector(IGlobalVector vector) 
			=> FreeDofIndexer.CheckCompatibleVector(vector);
	}
}
