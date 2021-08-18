using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Geometry.Tolerances;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;

//TODO: There is a lot of repetition between this FEM.Model and IGA.Model with regards to interconnection data. That code should 
//      be moved to a common class. Same goes for the interconnection methods of XSubdomain.
namespace MGroup.XFEM.Entities
{
	public class XModel<TElement> : IXModel where TElement: class, IXFiniteElement
	{
		private bool areDataStructuresConnected = false;

		public XModel(int dimension)
		{
			Dimension = dimension;

			// Standard dofs
			AllDofs.AddDof(StructuralDof.TranslationX);
			AllDofs.AddDof(StructuralDof.TranslationY);
			AllDofs.AddDof(StructuralDof.TranslationZ);
			AllDofs.AddDof(StructuralDof.RotationX);
			AllDofs.AddDof(StructuralDof.RotationY);
			AllDofs.AddDof(StructuralDof.RotationZ);
			AllDofs.AddDof(ThermalDof.Temperature);
			AllDofs.AddDof(PorousMediaDof.Pressure);
		}

		public ActiveDofs AllDofs { get; } = new ActiveDofs();

		public int Dimension { get; }

		public Dictionary<int, TElement> Elements { get; } = new Dictionary<int, TElement>();

		public Dictionary<int, EnrichmentItem> Enrichments { get; } = new Dictionary<int, EnrichmentItem>();

		public bool FindConformingSubcells { get; set; } = false;

		public IGeometryModel GeometryModel { get; set; }

		public IMeshTolerance MeshTolerance { get; set; } = new ArbitrarySideMeshTolerance();

		public List<IModelObserver> ModelObservers { get; set; } = new List<IModelObserver>();

		public List<Load> NodalLoads { get; private set; } = new List<Load>();

		public Dictionary<int, XNode> Nodes { get; } = new Dictionary<int, XNode>();

		public int NumSubdomains => Subdomains.Count;

		public Dictionary<int, XSubdomain<TElement>> Subdomains { get; } = new Dictionary<int, XSubdomain<TElement>>();

		public void ConnectDataStructures()
		{
			if (!areDataStructuresConnected)
			{
				BuildInterconnectionData();
				RemoveInactiveNodalLoads();
				areDataStructuresConnected = true;
			}
		}

		public IEnumerable<DirichletElementLoad> EnumerateDirichletBoundaryConditions(int subdomainID) //TODO: Duplicate code with FEM.Model
		{
			var subdomainLoads = new List<DirichletElementLoad>();
			foreach (IXFiniteElement element in Subdomains[subdomainID].Elements)
			{
				var load = new DirichletElementLoad(element);
				if (!load.IsZero())
				{
					subdomainLoads.Add(load);
				}
			}
			return subdomainLoads;
		}

		IEnumerable<IElement> IModel.EnumerateElements(int subdomainID) => Subdomains[subdomainID].Elements;

		public IEnumerable<TElement> EnumerateElements(int subdomainID) => Subdomains[subdomainID].Elements;

		//TODO: There must be a better way than recreating the data structures. Nope just remove this. Operating on all elements 
		//		should be done through IAlgebraicModel. 
		public IEnumerable<IXFiniteElement> EnumerateElements()
		{
			var result = new IXFiniteElement[Elements.Count];
			for (int i = 0; i < Elements.Count; ++i) result[i] = Elements[i];
			return result;
		}

		public IEnumerable<Load> EnumerateNodalLoads(int subdomainID)
		{
			//TODO: This partitioning should be done in ConnectDataStructures and then just return the correct collection.
			var subdomainLoads = new List<Load>();
			foreach (Load load in NodalLoads)
			{
				if (load.Node.Subdomains.Contains(subdomainID))
				{
					subdomainLoads.Add(load);
				}
			}
			return subdomainLoads;
		}

		public IEnumerable<INode> EnumerateNodes() => Nodes.Values;

		public IEnumerable<ISubdomain> EnumerateSubdomains() => Subdomains.Values;

		public INode GetNode(int nodeID) => Nodes[nodeID];

		public ISubdomain GetSubdomain(int subdomainID) => Subdomains[subdomainID];

		public void Initialize()
		{
			ConnectDataStructures();
			UpdateStatePrivate(true, null, null);
		}

		public void SaveMaterialState()
		{
			foreach (XSubdomain<TElement> subdomain in Subdomains.Values)
			{
				subdomain.SaveMaterialState();
			}
		}

		public void ScaleConstraints(double scalingFactor)
		{
			foreach (XNode node in Nodes.Values)
			{
				foreach (Constraint constraint in node.Constraints)
				{
					constraint.Amount *= scalingFactor;
				}
			}
		}

		/// <summary>
		/// 
		/// </summary>
		/// <param name="solutionFreeDofs">Total displacements of all dofs of each subdomain.</param>
		public void Update(IAlgebraicModel algebraicModel, IGlobalVector solutionFreeDofs)
		{
			UpdateStatePrivate(false, algebraicModel, solutionFreeDofs);
		}

		private void BuildInterconnectionData()
		{
			// Associate each element with its subdomains
			foreach (XSubdomain<TElement> subdomain in Subdomains.Values)
			{
				foreach (IXFiniteElement element in subdomain.Elements)
				{
					element.SetSubdomainID(subdomain.ID);
				}
			}

			// Associate each node with its elements
			foreach (IXFiniteElement element in Elements.Values)
			{
				foreach (XNode node in element.Nodes) node.ElementsDictionary[element.ID] = element;
			}

			// Associate each node with its subdomains
			foreach (XNode node in Nodes.Values)
			{
				foreach (IXFiniteElement element in node.ElementsDictionary.Values)
				{
					node.Subdomains.Add(element.SubdomainID);
				}
			}

			// Associate each subdomain with its nodes
			foreach (XSubdomain<TElement> subdomain in Subdomains.Values) subdomain.DefineNodesFromElements();
		}

		private void CalcConformingSubcells()
		{
			IConformingTriangulator triangulator;
			if (Dimension == 2) triangulator = new ConformingTriangulator2D();
			else if (Dimension == 3) triangulator = new ConformingTriangulator3D();
			else throw new NotImplementedException();

			foreach (IXFiniteElement element in Elements.Values)
			{
				var intersections = new List<IElementDiscontinuityInteraction>();
				foreach (IElementDiscontinuityInteraction interaction in element.InteractingDiscontinuities.Values)
				{
					if (interaction.RelativePosition == RelativePositionCurveElement.Intersecting)
					{
						intersections.Add(interaction);
					}
				}
				if (intersections.Count > 0)
				{
					element.ConformingSubcells = triangulator.FindConformingMesh(element, intersections, MeshTolerance);
				}
			}
		}

		private void RemoveInactiveNodalLoads()
		{
			// Static loads
			var activeLoadsStatic = new List<Load>(NodalLoads.Count);
			foreach (Load load in NodalLoads)
			{
				bool isConstrained = false;
				foreach (Constraint constraint in load.Node.Constraints)
				{
					if (load.DOF == constraint.DOF)
					{
						isConstrained = true;
						break;
					}
				}
				if (!isConstrained) activeLoadsStatic.Add(load);
			}
			NodalLoads = activeLoadsStatic;
		}

		/// <summary>
		/// Common operations for intializing/updating the model's state.
		/// </summary>
		/// <param name="firstAnalysis"></param>
		/// <param name="subdomainFreeDisplacements">if <paramref name="firstAnalysis"/> == true, this can be null.</param>
		private void UpdateStatePrivate(bool firstAnalysis, IAlgebraicModel algebraicModel, IGlobalVector solutionFreeDofs)
		{
			// Update the discontinuities
			if (firstAnalysis) GeometryModel.InitializeGeometry();
			else GeometryModel.UpdateGeometry(algebraicModel, solutionFreeDofs);
			GeometryModel.InteractWithMesh();

			// Optionally calculate conforming subcells for elements that interact with discontinuities
			if (FindConformingSubcells) CalcConformingSubcells();

			// Define enrichments and their dofs
			//TODO: The enrichments may need to change during the analysis (e.g. branching cracks, crack junctions, etc)
			if (firstAnalysis)
			{
				IEnumerable<EnrichmentItem> enrichments = GeometryModel.Enricher.DefineEnrichments();
				foreach (EnrichmentItem enrichment in enrichments)
				{
					this.Enrichments[enrichment.ID] = enrichment;
					foreach (IDofType dof in enrichment.EnrichedDofs)
					{
						AllDofs.AddDof(dof);
					}
				}
			}

			// Enrich the required nodes
			GeometryModel.Enricher.ApplyEnrichments();

			// Identify each element's dofs
			foreach (IXFiniteElement element in Elements.Values) element.IdentifyDofs();

			// Identify each element's integration points and the material properties at those points
			foreach (IXFiniteElement element in Elements.Values) element.IdentifyIntegrationPointsAndMaterials();

			// Let observers read the current state and update themselves
			foreach (IModelObserver observer in ModelObservers) observer.Update();
		}
	}
}
