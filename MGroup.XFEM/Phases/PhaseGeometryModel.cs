using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Entities;

//TODO: The order of operations is problematic when merging level sets, which is typically a geometric operation and thus
//      happens before geometry-mesh interaction. However merging level sets needs the interaction between phases and nodes.
//      Improve the current unsafe implementation.
namespace MGroup.XFEM.Phases
{
    public class PhaseGeometryModel : IGeometryModel
    {
        private readonly XModel<IXMultiphaseElement> physicalModel;
        private bool calcPhasesNodesInteractions = true;

        public PhaseGeometryModel(XModel<IXMultiphaseElement> physicalModel)
        {
            this.physicalModel = physicalModel;
        }

        public bool EnableOptimizations { get; set; } = false;

        public INodeEnricher Enricher { get; set; }

        public List<IPhaseGeometryObserver> GeometryObservers { get; } = new List<IPhaseGeometryObserver>();

        public List<IPhaseMeshInteractionObserver> InteractionObservers { get; } = new List<IPhaseMeshInteractionObserver>();

        public bool MergeOverlappingPhases { get; set; } = true;

        public Dictionary<int, IPhase> Phases { get; } = new Dictionary<int, IPhase>();
        
        public Dictionary<int, IPhaseBoundary> PhaseBoundaries { get; } = new Dictionary<int, IPhaseBoundary>();

        public IEnumerable<IXDiscontinuity> EnumerateDiscontinuities() => PhaseBoundaries.Values;

        public IXDiscontinuity GetDiscontinuity(int discontinuityID) => PhaseBoundaries[discontinuityID];

        public void InitializeGeometry()
        {
            //calcPhasesNodesInteractions = true;

            //foreach (IPhaseBoundary boundary in PhaseBoundaries.Values)
            //{
            //    boundary.InitializeGeometry();
            //}

            //if (MergeOverlappingPhases) MergePhases();


            foreach (IPhaseGeometryObserver observer in GeometryObservers) observer.Update();
        }

        public void InteractWithMesh()
        {
            // Phases - nodes
            InteractWithNodes();

            // Phases & phase boundaries - elements
            IPhase defaultPhase = Phases.Values.Where(p => p is DefaultPhase).FirstOrDefault();
            foreach (IPhase phase in Phases.Values)
            {
                if (phase != defaultPhase) phase.InteractWithElements(physicalModel.Elements);
            }
            if (defaultPhase != null) defaultPhase.InteractWithElements(physicalModel.Elements);

            foreach (IPhaseMeshInteractionObserver observer in InteractionObservers) observer.Update();
        }

        public void UpdateGeometry(Dictionary<int, Vector> subdomainFreeDisplacements)
        {
            calcPhasesNodesInteractions = true;

            foreach (IPhaseBoundary boundary in PhaseBoundaries.Values)
            {
                boundary.UpdateGeometry(subdomainFreeDisplacements);
            }

            if (MergeOverlappingPhases) MergePhases();

            foreach (IPhaseGeometryObserver observer in GeometryObservers) observer.Update();
        }

        private void InteractWithNodes()
        {
            if (calcPhasesNodesInteractions)
            {
                IPhase defaultPhase = Phases.Values.Where(p => p is DefaultPhase).FirstOrDefault();
                foreach (IPhase phase in Phases.Values)
                {
                    if (phase != defaultPhase) phase.InteractWithNodes(physicalModel.XNodes);
                }
                if (defaultPhase != null) defaultPhase.InteractWithNodes(physicalModel.XNodes);
            }
            calcPhasesNodesInteractions = false;
        }

        private void MergePhases()
        {
            InteractWithNodes(); // Do it beforehand since it will assist with geometric unions

            var unifiedPhases = new List<IPhase>();
            IPhase defaultPhase = null;
            foreach (IPhase phase in Phases.Values)
            {
                if (phase is DefaultPhase)
                {
                    defaultPhase = phase;
                    continue;
                }
                unifiedPhases.Add(phase);
            }

            int i = 0;
            while (i < unifiedPhases.Count - 1)
            {
                bool unionFound = false;
                for (int j = i + 1; j < unifiedPhases.Count; ++j)
                {
                    bool areJoined = unifiedPhases[i].UnionWith(unifiedPhases[j]);
                    if (areJoined)
                    {
                        unifiedPhases.RemoveAt(j);
                        unionFound = true;
                        break;
                    }
                }
                if (!unionFound)
                {
                    ++i;
                }
            }

            Phases.Clear();
            if (defaultPhase != null) Phases[defaultPhase.ID] = defaultPhase;
            foreach (IPhase phase in unifiedPhases) Phases[phase.ID] = phase;

            PhaseBoundaries.Clear();
            foreach (IPhase phase in Phases.Values)
            {
                foreach (IPhaseBoundary boundary in phase.AllBoundaries)
                {
                    PhaseBoundaries[boundary.ID] = boundary;
                }
            }
        }
    }
}
