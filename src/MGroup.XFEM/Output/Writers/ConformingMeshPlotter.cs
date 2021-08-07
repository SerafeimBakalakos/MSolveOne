using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
    public class ConformingMeshPlotter : IModelObserver
    {
        private readonly IXModel model;
        private readonly string outputDirectory;
        private int iteration;

        public ConformingMeshPlotter(string outputDirectory, IXModel model)
        {
            this.outputDirectory = outputDirectory;
            this.model = model;
            iteration = 0;
        }

        public void Update()
        {
            string path = Path.Combine(outputDirectory, $"conforming_mesh_t{iteration}.vtk");
            using (var writer = new VtkFileWriter(path))
            { 
                var conformingMesh = new ConformingOutputMesh(model);
                writer.WriteMesh(conformingMesh);
            }

            ++iteration;
        }
    }
}
