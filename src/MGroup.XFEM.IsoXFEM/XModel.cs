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
using MGroup.XFEM.Entities;
using MGroup.LinearAlgebra.Vectors;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Reduction;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.IsoXFEM.SolidOnlyTriangulator;
using MGroup.XFEM.IsoXFEM.IsoXfemElements;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Geometry.LSM.DualMesh;


//TODO: There is a lot of repetition between this FEM.Model and IGA.Model with regards to interconnection data. That code should 
//      be moved to a common class. Same goes for the interconnection methods of XSubdomain.
namespace MGroup.XFEM.IsoXFEM
{
	public class XModel<TElement> : IXModel where TElement : class, IIsoXfemElement
	{
		public Vector sizesOfElements;
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

		public Vector relativeCriteria;

		public IXGeometryDescription levelSetDescription;
		public IDualMesh Mesh { get; set; }
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

		public IEnumerable<DirichletElementLoad> EnumerateDirichletBoundaryConditions(int subdomainID)  
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
			//UpdateStatePrivate(true, null, null);
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
			//if (Dimension == 3)
			//{
			//	levelSetDescription = new DualMeshLsmSymmetric(0, relativeCriteria.CopyToArray(), (DualCartesianSimplicialSymmetricMeshBase)Mesh);
			//	//levelSetDescription = new SimpleLsm3D(0, relativeCriteria.CopyToArray());
			//	foreach (var element in Elements)
			//	{
			//		var intersection = levelSetDescription.Intersect(element.Value);
			//		element.Value.RegisterInteractionWithLsm3D(intersection);
			//	}
			//}
			//else
			//{
			//	//var geometry = new Circle2D(0, 0, 0);
			//	//var levelSetStorage = new LsmStorageRaw(Dimension, relativeCriteria.CopyToArray());
			//	levelSetDescription = new DualMeshLsmSymmetric(0, relativeCriteria.CopyToArray(), (DualCartesianSimplicialSymmetricMeshBase)Mesh);
			//	//levelSetDescription = new SimpleLsm3D(0, relativeCriteria.CopyToArray());
			//	foreach (var element in Elements)
			//	{
			//		#region Debug
			//		//if (element.Value.ID == 740)
			//		//{
			//		//	Console.WriteLine();
			//		//}
			//		#endregion
			//		var intersection = levelSetDescription.Intersect(element.Value);
			//		if (intersection.RelativePosition== RelativePositionCurveElement.Intersecting)
			//		{
			//			element.Value.RegisterInteractionWithLsm2D(intersection);
			//		}					
			//	}
			//}
			CalcConformingSubcells();
			int i = 0;
			foreach(var element in Elements.Values)
			{
				element.StiffnessMatrix(element);
				sizesOfElements[i] = element.SizeOfElement;
				i++;
			}
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
			ISolidOnlyTriangulator triangulator;
			if (Dimension == 2) triangulator = new SolidOnlyTriangulator2D();
			else if (Dimension == 3) triangulator = new SolidOnlyTriangulator3D();
			else throw new NotImplementedException();

			foreach (IIsoXfemElement element in Elements.Values)
			{
				int[] connectionOfElement = new int[element.Nodes.Count];
				int i = 0;
				foreach (var node in element.Nodes)
				{
					connectionOfElement[i] = node.ID;
					i++;
				}
				Vector elementRelativeCriteria = relativeCriteria.GetSubvector(connectionOfElement);
				element.ElementLevelSet = elementRelativeCriteria;
				#region Debug
				//if (element.ID==740)
				//{
				//	Console.WriteLine();
				//}
				#endregion
				element.DefinePhaseOfElement();
				if (element.PhaseElement == IIsoXfemElement.Phase.boundaryElement)
				{
					if (Dimension == 3)
					{
						//var intersections = new List<IElementDiscontinuityInteraction>();
						//foreach (IElementDiscontinuityInteraction interaction in element.InteractingDiscontinuities.Values)
						//{
						//	//if (interaction.RelativePosition == RelativePositionCurveElement.Intersecting)
						//	//{
						//	intersections.Add(interaction);
						//	//}
						//}
						//IMeshTolerance meshTolerance = new MinimumSideMeshTolerance();
						////IMeshTolerance meshTolerance = new UserDefinedMeshTolerance(Elements.First().Value.CalcBulkSizeCartesian());
						//triangulator.LevelSetDescription = levelSetDescription;
						//element.ConformingSubcells = triangulator.FindConformingMesh(element, intersections, meshTolerance);
						triangulator.NodalLevelSetModel = relativeCriteria;
						triangulator.ElementNodalLevelSetValues = element.ElementLevelSet;
						element.ConformingSubcells = triangulator.FindConformingMesh(element, null, null);
					}
					else if (Dimension == 2)
					{
						//var intersections = new List<IElementDiscontinuityInteraction>();
						//foreach (IElementDiscontinuityInteraction interaction in element.InteractingDiscontinuities.Values)
						//{
						//	//if (interaction.RelativePosition == RelativePositionCurveElement.Intersecting)
						//	//{
						//	intersections.Add(interaction);
						//	//}
						//}
						//IMeshTolerance meshTolerance = new MinimumSideMeshTolerance();
						////IMeshTolerance meshTolerance = new UserDefinedMeshTolerance(Elements.First().Value.CalcBulkSizeCartesian());
						//triangulator.LevelSetDescription = levelSetDescription;
						//element.ConformingSubcells = triangulator.FindConformingMesh(element, intersections, meshTolerance);
						triangulator.ElementNodalLevelSetValues = element.ElementLevelSet;
						element.ConformingSubcells = triangulator.FindConformingMesh(element, null, null);
					}
					else throw new NotImplementedException();
					var sizeofelement = 0.0;
					foreach (var subcell in element.ConformingSubcells)
					{
						(var centroid, var sizesubcell) = subcell.FindCentroidAndBulkSizeCartesian(element);
						sizeofelement += Math.Abs(sizesubcell);
					}
					element.SizeOfElement = sizeofelement;
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

	}
}
