using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
    public class PhaseInteractingElementsPlotter : IModelObserver
    {
        private readonly IXModel model;
        private readonly string outputDirectory;
        private int iteration;

        public PhaseInteractingElementsPlotter(string outputDirectory, IXModel model)
        {
            this.outputDirectory = outputDirectory;
            this.model = model;
            iteration = 0;
        }

        public void Update()
        {
            var mesh = new OutputMesh();
            foreach (IXFiniteElement element in model.EnumerateElements())
            {
                if (element.InteractingDiscontinuities.Count == 0) continue;
                foreach (int phaseID in element.InteractingDiscontinuities.Keys)
                {
                    mesh.AddElement(element, phaseID);
                }
            }

            string path = Path.Combine(outputDirectory, $"intersected_elements_t{iteration}.vtk");
            using (var writer = new VtkFileWriter(path))
            {
                writer.WriteMesh(mesh);
                writer.WriteScalarField("phase", mesh, mesh.PointValues);
            }

            ++iteration;
        }

        private class OutputMesh : IOutputMesh
        {
            private readonly List<VtkCell> cells = new List<VtkCell>();
            private readonly List<VtkPoint> vertices  = new List<VtkPoint>();

            public List<double> PointValues { get; } = new List<double>();

            public void AddElement(IXFiniteElement element, int intersectingPhaseID)
            {
                var verticesOfCell = new VtkPoint[element.Nodes.Count]; 
                for (int n = 0; n < element.Nodes.Count; ++n)
                {
                    verticesOfCell[n] = new VtkPoint(this.vertices.Count, element.Nodes[n].Coordinates);
                    this.vertices.Add(verticesOfCell[n]);
                    this.PointValues.Add(intersectingPhaseID);
                }
                cells.Add(new VtkCell(element.CellType, verticesOfCell));
            }

            public int NumOutCells => cells.Count;

            public int NumOutVertices => vertices.Count;

            public IEnumerable<VtkCell> OutCells => cells;

            public IEnumerable<VtkPoint> OutVertices => vertices;
        }
    }
}
