using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Vtk;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Output.Writers
{
    public class NodalPhasesPlotter : IPhaseMeshInteractionObserver
    {
        private readonly double colorForDefaultPhase;
        private readonly XModel<IXMultiphaseElement> model;
        private readonly string outputDirectory;

        private int iteration;

        public NodalPhasesPlotter(string outputDirectory, XModel<IXMultiphaseElement> model,
            double colorForDefaultPhase)
        {
            this.outputDirectory = outputDirectory;
            this.model = model;
            this.colorForDefaultPhase = colorForDefaultPhase;

            iteration = 0;
        }

        public NodalPhasesPlotter(string outputDirectory, XModel<IXMultiphaseElement> model)
            : this(outputDirectory, model, DefaultPhase.defaultPhaseID)
        {
        }

        public void Update()
        {
            string path = Path.Combine(outputDirectory, $"nodal_phases_t{iteration}.vtk");
            using (var writer = new VtkPointWriter(path))
            {
                var nodalPhases = new Dictionary<INode, double>();

                foreach (XNode node in model.XNodes)
                {
                    double phaseID = node.PhaseID;
                    if (node.PhaseID == DefaultPhase.defaultPhaseID) phaseID = colorForDefaultPhase;
                    nodalPhases[node] = phaseID;
                }

                writer.WriteScalarField("nodal_phases", nodalPhases);
            }

            ++iteration;
        }
    }
}
