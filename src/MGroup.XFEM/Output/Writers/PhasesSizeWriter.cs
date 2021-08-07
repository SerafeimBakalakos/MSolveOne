using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Output.Fields;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Output.Writers
{
    public class PhasesSizeWriter : IModelObserver
    {
        private readonly PhaseGeometryModel geometryModel;
        private readonly string outputDirectory;
        private readonly XModel<IXMultiphaseElement> physicalModel;
        private int iteration;

        public PhasesSizeWriter(string outputDirectory, 
            XModel<IXMultiphaseElement> physicalModel, PhaseGeometryModel geometryModel)
        {
            this.outputDirectory = outputDirectory;
            this.physicalModel = physicalModel;
            this.geometryModel = geometryModel;

            iteration = 0;
        }

        public void Update()
        {
            var volumes = Utilities.CalcBulkSizeOfEachPhase(physicalModel, geometryModel);
            string path = Path.Combine(outputDirectory, $"phase_sizes_t{iteration}.txt");
            using (var writer = new StreamWriter(path))
            {
                var builder = new StringBuilder();
                writer.WriteLine("Total areas/volumes of each material phase:");
                writer.WriteLine("Phase Size");
                foreach (int phase in volumes.Keys)
                {
                    writer.WriteLine($"{phase} {volumes[phase]}");
                }
                writer.Flush();
            }

            ++iteration;
        }
    }
}
