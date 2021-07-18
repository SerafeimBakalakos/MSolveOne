using System.Collections.Generic;
using MGroup.FEM.Interfaces;
using System.Linq;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization.Loads;
using System;
//

//TODO: find what is going on with the dynamic loads and refactor them. That 564000000 in AssignMassAccelerationHistoryLoads()
//      cannot be correct.
//TODO: ConnectDataStructures() should not be called twice. There should be a flag that determines if it has been called. If it
//      has, the method should just return without doing anything.
//TODO: Replace all IList with IReadOnlyList. Even better, have a different class to create the model than the one used to 
//      store the entities, so that they can be accessed by solvers, analyzers & loggers. Could the latter be the same for FEM, 
//      IGA, XFEM?
namespace MGroup.FEM.Entities
{
	public class Model : IModel, ITransientModel
	{
		//public IList<EmbeddedNode> EmbeddedNodes { get; } = new List<EmbeddedNode>();

		public ActiveDofs AllDofs { get; } = new ActiveDofs();

		public IList<Cluster> Clusters => ClustersDictionary.Values.ToList();
		public Dictionary<int, Cluster> ClustersDictionary { get; } = new Dictionary<int, Cluster>();

		public Dictionary<int, Element> ElementsDictionary { get; } = new Dictionary<int, Element>();

		public IList<ElementMassAccelerationHistoryLoad> ElementMassAccelerationHistoryLoads { get; }
			= new List<ElementMassAccelerationHistoryLoad>();
		public IList<ElementMassAccelerationLoad> ElementMassAccelerationLoads { get; }
			= new List<ElementMassAccelerationLoad>();
		public IList<Load> Loads { get; private set; } = new List<Load>();
		public IList<MassAccelerationLoad> MassAccelerationLoads { get; } = new List<MassAccelerationLoad>();
		public IList<IMassAccelerationHistoryLoad> MassAccelerationHistoryLoads { get; } = new List<IMassAccelerationHistoryLoad>();

		public Dictionary<int, Node> NodesDictionary { get; } = new Dictionary<int, Node>();

		public int NumSubdomains => SubdomainsDictionary.Count;

		public Dictionary<int, Subdomain> SubdomainsDictionary { get; } = new Dictionary<int, Subdomain>();

		public int TimeStep { get; set; }

		public IList<TransientNodalLoad> TransientNodalLoads { get; private set; } = new List<TransientNodalLoad>();

		//What is the purpose of this method? If someone wanted to clear the Model, they could just create a new one.
		public void Clear()
		{
			Loads.Clear();
			ClustersDictionary.Clear();
			SubdomainsDictionary.Clear();
			ElementsDictionary.Clear();
			NodesDictionary.Clear();
			ElementMassAccelerationHistoryLoads.Clear();
			ElementMassAccelerationLoads.Clear();
			MassAccelerationHistoryLoads.Clear();
			MassAccelerationLoads.Clear();
		}

		// Warning: This is called by the analyzer, so that the user does not have to call it explicitly. However, it is must be 
		// called explicitly before the AutomaticDomainDecompositioner is used.
		public void ConnectDataStructures()
		{
			BuildInterconnectionData();
			RemoveInactiveNodalLoads();

			//TODOSerafeim: This should be called by the analyzer, which defines when the dofs are ordered and when the global vectors/matrices are built.
			//AssignLoads();
		}

		public IEnumerable<DirichletElementLoad> EnumerateDirichletBoundaryConditions(int subdomainID)
		{
			var subdomainLoads = new List<DirichletElementLoad>();
			foreach (Element element in SubdomainsDictionary[subdomainID].Elements)
			{
				var load = new DirichletElementLoad(element);
				if (!load.IsZero())
				{
					subdomainLoads.Add(load);
				}
			}
			return subdomainLoads;
		}


		public IEnumerable<IElement> EnumerateElements(int subdomainID) => SubdomainsDictionary[subdomainID].Elements;

		public IEnumerable<SerafeimsAwesomeElementAccelerationLoad> EnumerateMassAccelerationLoads(int subdomainID)
		{
			var subdomainLoads = new List<SerafeimsAwesomeElementAccelerationLoad>();
			if (MassAccelerationLoads.Count > 0)
			{
				Subdomain subdomain = SubdomainsDictionary[subdomainID];
				foreach (Element element in subdomain.Elements)
				{
					var load = new SerafeimsAwesomeElementAccelerationLoad()
					{
						Element = element,
						GlobalLoads = MassAccelerationLoads
					};
				}
			}
			return subdomainLoads;
		}

		public IEnumerable<SerafeimsAwesomeElementAccelerationLoad> EnumerateElementMassLoads(int subdomainID)
		{
			var subdomainLoads = new List<SerafeimsAwesomeElementAccelerationLoad>();
			foreach (ElementMassAccelerationLoad load in ElementMassAccelerationLoads)
			{
				if (load.Element.SubdomainID != subdomainID)
				{
					continue;
				}

				var newLoad = new SerafeimsAwesomeElementAccelerationLoad()
				{
					Element = load.Element,
					GlobalLoads = this.MassAccelerationLoads
				};
				subdomainLoads.Add(newLoad);
			}
			return subdomainLoads;
		}

		public IEnumerable<SerafeimsAwesomeElementAccelerationLoad> EnumerateMassAccelerationHistoryLoads(int subdomainID)
		{
			var subdomainLoads = new List<SerafeimsAwesomeElementAccelerationLoad>();

			if (MassAccelerationHistoryLoads.Count > 0)
			{
				var m = new List<MassAccelerationLoad>(MassAccelerationHistoryLoads.Count);
				foreach (IMassAccelerationHistoryLoad l in MassAccelerationHistoryLoads)
				{
					m.Add(new MassAccelerationLoad() { Amount = l[TimeStep], DOF = l.DOF });
				}

				Subdomain subdomain = SubdomainsDictionary[subdomainID];
				foreach (IElement element in subdomain.Elements)
				{
					var load = new SerafeimsAwesomeElementAccelerationLoad()
					{
						Element = element,
						GlobalLoads = m
					};
					subdomainLoads.Add(load);
				}
			}

			foreach (ElementMassAccelerationHistoryLoad load in ElementMassAccelerationHistoryLoads)
			{
				if (load.Element.SubdomainID != subdomainID)
				{
					continue;
				}

				MassAccelerationLoad hl = new MassAccelerationLoad()
				{
					Amount = load.HistoryLoad[TimeStep] * 564000000,
					DOF = load.HistoryLoad.DOF
				};

				var newLoad = new SerafeimsAwesomeElementAccelerationLoad()
				{
					Element = load.Element,
					GlobalLoads = new MassAccelerationLoad[] { hl }
				};
				subdomainLoads.Add(newLoad);
			}

			return subdomainLoads;
		}

		public IEnumerable<Load> EnumerateNodalLoads(int subdomainID)
		{
			//TODO: This partitioning should be done in ConnectDataStructures and then just return the correct collection.
			var subdomainLoads = new List<Load>();
			foreach (Load load in Loads)
			{
				if (load.Node.Subdomains.Contains(subdomainID))
				{
					subdomainLoads.Add(load);
				}
			}
			return subdomainLoads;
		}

		public IEnumerable<TransientNodalLoad> EnumerateTransientNodalLoads(int subdomainID)
		{
			//TODO: This partitioning should be done in ConnectDataStructures and then just return the correct collection.
			var subdomainLoads = new List<TransientNodalLoad>();
			foreach (TransientNodalLoad load in TransientNodalLoads)
			{
				if (load.Node.Subdomains.Contains(subdomainID))
				{
					subdomainLoads.Add(load);
				}
			}
			return subdomainLoads;
		}

		public IEnumerable<INode> EnumerateNodes() => NodesDictionary.Values;

		public IEnumerable<ISubdomain> EnumerateSubdomains() => SubdomainsDictionary.Values;

		public INode GetNode(int nodeID) => NodesDictionary[nodeID];

		public ISubdomain GetSubdomain(int subdomainID) => SubdomainsDictionary[subdomainID];

		public void SaveMaterialState()
		{
			foreach (Subdomain subdomain in SubdomainsDictionary.Values)
			{
				subdomain.SaveMaterialState();
			}
		}

		public void ScaleConstraints(double scalingFactor)
		{
			foreach (Node node in NodesDictionary.Values)
			{
				foreach (Constraint constraint in node.Constraints)
				{
					constraint.Amount *= scalingFactor;
				}
			}
		}

		private void BuildElementDictionaryOfEachNode()
		{
			foreach (Element element in ElementsDictionary.Values)
			{
				foreach (Node node in element.Nodes) node.ElementsDictionary[element.ID] = element;
			}
		}

		private void BuildInterconnectionData()//TODOMaria: maybe I have to generate the constraints dictionary for each subdomain here
		{
			BuildSubdomainOfEachElement();
			DuplicateInterSubdomainEmbeddedElements();
			BuildElementDictionaryOfEachNode();
			foreach (Node node in NodesDictionary.Values) node.FindAssociatedSubdomains();

			//BuildNonConformingNodes();

			foreach (Subdomain subdomain in SubdomainsDictionary.Values) subdomain.DefineNodesFromElements();
		}

		private void BuildSubdomainOfEachElement()
		{
			foreach (Subdomain subdomain in SubdomainsDictionary.Values)
			{
				foreach (Element element in subdomain.Elements)
				{
					element.SubdomainID = subdomain.ID;
				}
			}
		}

		private void BuildNonConformingNodes()
		{
			List<int> subIDs = new List<int>();
			foreach (Element element in ElementsDictionary.Values)
			{
				subIDs.Clear();

				foreach (Node node in element.Nodes)
				{
					foreach (int subID in node.Subdomains)
					{
						if (!subIDs.Contains(subID)) subIDs.Add(subID);

					}
				}

				foreach (Node node in element.Nodes)
				{
					foreach (int subID in subIDs)
					{
						if (!node.Subdomains.Contains(subID))
						{
							node.NonMatchingSubdomainsDictionary.Add(subID, SubdomainsDictionary[subID]);
						}
					}
				}

			}
		}

		private void DuplicateInterSubdomainEmbeddedElements()
		{
			foreach (var e in ElementsDictionary.Values.Where(x => x.ElementType is IEmbeddedElement))
			{
				var subIDs = ((IEmbeddedElement)e.ElementType).EmbeddedNodes.Select(x => x.EmbeddedInElement.SubdomainID).Distinct();
				foreach (var s in subIDs.Where(x => x != e.SubdomainID))
				{
					SubdomainsDictionary[s].Elements.Add(e);
				}
			}
		}

		private void RemoveInactiveNodalLoads()
		{
			// Static loads
			var activeLoadsStatic = new List<Load>(Loads.Count);
			foreach (Load load in Loads)
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
			Loads = activeLoadsStatic;

			// Dynamic loads
			var activeLoadsDynamic = new List<TransientNodalLoad>(TransientNodalLoads.Count);
			foreach (TransientNodalLoad load in TransientNodalLoads)
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
				if (!isConstrained) activeLoadsDynamic.Add(load);
			}
			TransientNodalLoads = activeLoadsDynamic;
		}
	}
}
