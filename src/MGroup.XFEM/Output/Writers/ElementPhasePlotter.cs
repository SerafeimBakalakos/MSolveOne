using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Phases;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
    public class ElementPhasePlotter : IModelObserver
    {
        private readonly double colorForDefaultPhase;
        private readonly int defaultPhaseID;
        private readonly PhaseGeometryModel geometricModel;
        private readonly XModel<IXMultiphaseElement> physicalModel;
        private readonly string outputDirectory;

        private int iteration;

        public ElementPhasePlotter(string outputDirectory, XModel<IXMultiphaseElement> physicalModel, 
            PhaseGeometryModel geometricModel, int defaultPhaseID, double colorForDefaultPhase)
        {
            this.outputDirectory = outputDirectory;
            this.physicalModel = physicalModel;
            this.geometricModel = geometricModel;
            this.defaultPhaseID = defaultPhaseID;
            this.colorForDefaultPhase = colorForDefaultPhase;

            iteration = 0;
        }

        public ElementPhasePlotter(string outputDirectory, XModel<IXMultiphaseElement> physicalModel,
            PhaseGeometryModel geometricModel, int defaultPhaseID) : 
            this(outputDirectory, physicalModel, geometricModel, defaultPhaseID, defaultPhaseID)
        {
        }

        public void Update()
        {
            var conformingMesh = new ConformingOutputMesh(physicalModel);
            Dictionary<VtkPoint, double> phases = FindPhasesOfElements(conformingMesh);
            string path = Path.Combine(outputDirectory, $"element_phases_t{iteration}.vtk");
            using (var writer = new VtkFileWriter(path))
            {
                writer.WriteMesh(conformingMesh);
                writer.WriteScalarField("phase", conformingMesh, v => phases[v]);
            }

            iteration++;
        }

        private Dictionary<VtkPoint, double> FindPhasesOfElements(ConformingOutputMesh conformingMesh)
        {
            var field = new Dictionary<VtkPoint, double>();
            foreach (IXMultiphaseElement element in physicalModel.Elements.Values)
            {
                var elementPhases = element.Phases;
                if (elementPhases.Count == 1)
                {
                    double phaseID = elementPhases.First().ID;
                    if (elementPhases.First() is DefaultPhase) phaseID = colorForDefaultPhase;
                    VtkCell outCell = conformingMesh.GetOutCellsForOriginal(element).First();
                    for (int n = 0; n < element.Nodes.Count; ++n) field[outCell.Vertices[n]] = phaseID;
                }
                else
                {
                    IEnumerable<ConformingOutputMesh.Subcell> subcells = conformingMesh.GetSubcellsForOriginal(element);
                    foreach (ConformingOutputMesh.Subcell subcell in subcells)
                    {
                        Debug.Assert(subcell.OutVertices.Count == 3 || subcell.OutVertices.Count == 4); //TODO: Not sure what happens for 2nd order elements

                        // TODO: Perhaps I should do the next operations in the natural system of the element.
                        // Find the centroid
                        double[] centroidNatural = subcell.OriginalSubcell.FindCentroidNatural();
                        var centroid = new XPoint(centroidNatural.Length);
                        centroid.Element = subcell.ParentElement;
                        centroid.Coordinates[CoordinateSystem.ElementNatural] = centroidNatural;
                        centroid.ShapeFunctions = centroid.Element.Interpolation.EvaluateFunctionsAt(centroidNatural);

                        // Find the phase of the centroid
                        double phaseID = colorForDefaultPhase;
                        foreach (IPhase phase in elementPhases)
                        {
                            if (phase.ID == defaultPhaseID) continue;
                            if (phase.Contains(centroid))
                            {
                                phaseID = phase.ID;
                                break;
                            }
                        }

                        // All vertices of the subceel will be assigned the same phase as the centroid
                        for (int v = 0; v < subcell.OutVertices.Count; ++v)
                        {
                            VtkPoint vertexOut = subcell.OutVertices[v];
                            field[vertexOut] = phaseID;
                        }
                    }
                }
            }
            return field;
        }
    }
}
