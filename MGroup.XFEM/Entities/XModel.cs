using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Geometry.Tolerances;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization.Loads;

//TODO: There is a lot of repetition between this FEM.Model and IGA.Model with regards to interconnection data. That code should 
//      be moved to a common class. Same goes for the interconnection methods of XSubdomain.
namespace MGroup.XFEM.Entities
{
    public class XModel<TElement> : IXModel where TElement: IXFiniteElement
    {
        private bool areDataStructuresConnected = false;

        private List<IEnrichmentObserver> enrichmentObservers = new List<IEnrichmentObserver>();

        public XModel(int dimension)
        {
            Dimension = dimension;
        }

        public Table<INode, IDofType, double> Constraints { get; private set; } = new Table<INode, IDofType, double>();

        public int Dimension { get; }

        IReadOnlyList<IElement> IModel.Elements
        {
            get
            {
                var result = new IElement[Elements.Count];
                for (int i = 0; i < Elements.Count; ++i) result[i] = Elements[i];
                return result;
            }
        }

        public List<TElement> Elements { get; } = new List<TElement>();

        public Dictionary<int, EnrichmentItem> Enrichments { get; } = new Dictionary<int, EnrichmentItem>();

        public bool FindConformingSubcells { get; set; } = false;

        public IGeometryModel GeometryModel { get; set; }

        public IGlobalFreeDofOrdering GlobalDofOrdering { get; set; }

        public IMeshTolerance MeshTolerance { get; set; } = new ArbitrarySideMeshTolerance();

        public List<IModelObserver> ModelObservers { get; set; } = new List<IModelObserver>();

        public List<NodalLoad> NodalLoads { get; private set; } = new List<NodalLoad>();

        IReadOnlyList<INode> IModel.Nodes => XNodes;
        public List<XNode> XNodes { get; } = new List<XNode>();

        IReadOnlyList<ISubdomain> IModel.Subdomains => Subdomains.Values.ToList();
        public Dictionary<int, XSubdomain> Subdomains { get; } = new Dictionary<int, XSubdomain>();

        public IList<IMassAccelerationHistoryLoad> MassAccelerationHistoryLoads => throw new NotImplementedException();

        IList<IMassAccelerationHistoryLoad> IModel.MassAccelerationHistoryLoads => throw new NotImplementedException();

        public void AssignLoads(NodalLoadsToSubdomainsDistributor distributeNodalLoads)
        {
            foreach (XSubdomain subdomain in Subdomains.Values) subdomain.Forces.Clear();
            AssignNodalLoads(distributeNodalLoads);
        }

        public void AssignNodalLoads(NodalLoadsToSubdomainsDistributor distributeNodalLoads)
        {
            var globalNodalLoads = new Table<INode, IDofType, double>();
            foreach (NodalLoad load in NodalLoads) globalNodalLoads.TryAdd(load.Node, load.DofType, load.Value);

            Dictionary<int, SparseVector> subdomainNodalLoads = distributeNodalLoads(globalNodalLoads);
            foreach (var idSubdomainLoads in subdomainNodalLoads)
            {
                Subdomains[idSubdomainLoads.Key].Forces.AddIntoThis(idSubdomainLoads.Value);
            }
        }

        public void AssignMassAccelerationHistoryLoads(int timeStep) => throw new NotImplementedException();

        public void ConnectDataStructures()
        {
            if (!areDataStructuresConnected)
            {
                BuildInterconnectionData();
                AssignConstraints();
                RemoveInactiveNodalLoads();
                areDataStructuresConnected = true;
            }
        }

        public IEnumerable<IXFiniteElement> EnumerateElements() 
        {
            //TODO: There must be a better way than recreating the data structures
            var result = new IXFiniteElement[Elements.Count];
            for (int i = 0; i < Elements.Count; ++i) result[i] = Elements[i];
            return result;
        }

        public void Initialize()
        {
            ConnectDataStructures();
            UpdateStatePrivate(true, null);
        }

        public void RegisterEnrichmentObserver(IEnrichmentObserver observer)
        {
            var previous = observer.RegisterAfterThese();
            foreach (IEnrichmentObserver other in previous)
            {
                if (!enrichmentObservers.Contains(other))
                {
                    if (other.RegisterAfterThese().Length == 0) enrichmentObservers.Add(other);
                    else
                    {
                        throw new ArgumentException("This observer depends on others that in turn depend on even more."
                            + " The order of registration cannot be safely determined automatically."
                            + " Please register them in the correct order yourself.");
                    }
                }
            }
            enrichmentObservers.Add(observer);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="subdomainFreeDisplacements">Total displacements of all dofs of each subdomain.</param>
        public void Update(Dictionary<int, Vector> subdomainFreeDisplacements)
        {
            UpdateStatePrivate(false, subdomainFreeDisplacements);
        }

        private void AssignConstraints()
        {
            foreach (XNode node in XNodes)
            {
                if (node.Constraints == null) continue;
                foreach (Constraint constraint in node.Constraints) Constraints[node, constraint.DOF] = constraint.Amount;
            }

            foreach (XSubdomain subdomain in Subdomains.Values) subdomain.ExtractConstraintsFromGlobal(Constraints);
        }

        private void BuildInterconnectionData()
        {
            // Associate each element with its subdomains
            foreach (XSubdomain subdomain in Subdomains.Values)
            {
                foreach (IXFiniteElement element in subdomain.Elements) element.Subdomain = subdomain;
            }

            // Associate each node with its elements
            foreach (IXFiniteElement element in Elements)
            {
                foreach (XNode node in element.Nodes) node.ElementsDictionary[element.ID] = element;
            }

            // Associate each node with its subdomains
            foreach (XNode node in XNodes)
            {
                foreach (IXFiniteElement element in node.ElementsDictionary.Values)
                {
                    node.SubdomainsDictionary[element.Subdomain.ID] = element.Subdomain;
                }
            }

            // Associate each subdomain with its nodes
            foreach (XSubdomain subdomain in Subdomains.Values) subdomain.DefineNodesFromElements();
        }

        private void CalcConformingSubcells()
        {
            IConformingTriangulator triangulator;
            if (Dimension == 2) triangulator = new ConformingTriangulator2D();
            else if (Dimension == 3) triangulator = new ConformingTriangulator3D();
            else throw new NotImplementedException();

            foreach (IXFiniteElement element in Elements)
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
            var activeLoadsStatic = new List<NodalLoad>(NodalLoads.Count);
            foreach (NodalLoad load in NodalLoads)
            {
                bool isConstrained = Constraints.Contains(load.Node, load.DofType);
                if (!isConstrained) activeLoadsStatic.Add(load);
            }
            NodalLoads = activeLoadsStatic;
        }

        /// <summary>
        /// Common operations for intializing/updating the model's state.
        /// </summary>
        /// <param name="firstAnalysis"></param>
        /// <param name="subdomainFreeDisplacements">if <paramref name="firstAnalysis"/> == true, this can be null.</param>
        private void UpdateStatePrivate(bool firstAnalysis, Dictionary<int, Vector> subdomainFreeDisplacements)
        {
            // Update the discontinuities
            if (firstAnalysis) GeometryModel.InitializeGeometry();
            else GeometryModel.UpdateGeometry(subdomainFreeDisplacements);
            //GeometryModel.InteractWithMesh();

            //// Optionally calculate conforming subcells for elements that interact with discontinuities
            //if (FindConformingSubcells) CalcConformingSubcells();

            //// Define enrichments and their dofs
            ////TODO: The enrichments may need to change during the analysis (e.g. branching cracks, crack junctions, etc)
            //if (firstAnalysis)
            //{
            //    IEnumerable<EnrichmentItem> enrichments = GeometryModel.Enricher.DefineEnrichments();
            //    foreach (EnrichmentItem enrichment in enrichments)
            //    {
            //        this.Enrichments[enrichment.ID] = enrichment;
            //    }
            //}

            //// Enrich the required nodes
            //GeometryModel.Enricher.ApplyEnrichments();

            //// Identify each element's dofs
            //foreach (IXFiniteElement element in Elements) element.IdentifyDofs();

            //// Identify each element's integration points and the material properties at those points
            //foreach (IXFiniteElement element in Elements) element.IdentifyIntegrationPointsAndMaterials();

            //// Let observers read the current state and update themselves
            //foreach (IModelObserver observer in ModelObservers) observer.Update();
            //foreach (IEnrichmentObserver observer in enrichmentObservers) observer.Update(Enrichments.Values);
        }
    }
}
