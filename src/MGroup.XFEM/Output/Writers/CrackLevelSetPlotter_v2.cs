using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.FriesHybridCrack
{
    public class CrackLevelSetPlotter_v2 : ICrackObserver
    {
        private readonly IXModel model;
        private readonly IHybridFriesCrackDescription crack;
        private readonly string outputDirectory;
        private int iteration;

        public CrackLevelSetPlotter_v2(IXModel model, IHybridFriesCrackDescription crack, string outputDirectory)
        {
            this.model = model;
            this.crack = crack;
            this.outputDirectory = outputDirectory;
            iteration = 0;
        }

        public void Update()
        {
            string path = $"{outputDirectory}\\crack_level_sets_{crack.ID}_t{iteration}.vtk";
            using (var writer = new VtkFileWriter(path))
            {
                var outputMesh = new ContinuousOutputMesh(model.Nodes.Values, model.EnumerateElements());
                writer.WriteMesh(outputMesh);
                
                writer.WriteScalarField("phi", outputMesh, v => crack.GetNodalLevelSets(model.Nodes[v.ID])[0]);
                writer.WriteScalarField("psi", outputMesh, v => crack.GetNodalLevelSets(model.Nodes[v.ID])[1]);
            }

            ++iteration;
        }
    }
}
