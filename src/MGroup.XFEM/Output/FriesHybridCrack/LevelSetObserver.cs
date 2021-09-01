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
    public class LevelSetObserver : ICrackObserver
    {
        private readonly IXModel model;
        private readonly IHybridFriesCrackDescription crack;
        private readonly string outputDirectory;
        private int iteration;

        public LevelSetObserver(IXModel model, IHybridFriesCrackDescription crack, string outputDirectory)
        {
            this.model = model;
            this.crack = crack;
            this.outputDirectory = outputDirectory;
            iteration = 0;
        }

        public void Update()
        {
            // Normals of vertices
            string path = $"{outputDirectory}\\level_sets_{crack.ID}_t{iteration}.vtk";
            using (var writer = new VtkFileWriter(path))
            {
                var outputMesh = new ContinuousOutputMesh(model.Nodes.Values, model.EnumerateElements());
                writer.WriteMesh(outputMesh);

                //var phi1 = new double[model.Nodes.Count];
                //var phi2 = new double[model.Nodes.Count];
                //var phi3 = new double[model.Nodes.Count];
                //for (int n = 0; n < model.Nodes.Count; ++n)
                //{
                //    XNode node = model.XNodes[n];
                //    double[] levelSets = crack.GetLevelSetsOf(node);
                //    phi1[n] = levelSets[0];
                //    phi2[n] = levelSets[1];
                //    phi3[n] = levelSets[2];
                //}
                
                writer.WriteScalarField("phi1", outputMesh, v => crack.GetLevelSetsOf(model.Nodes[v.ID])[0]);
                writer.WriteScalarField("phi2", outputMesh, v => crack.GetLevelSetsOf(model.Nodes[v.ID])[1]);
                writer.WriteScalarField("phi3", outputMesh, v => crack.GetLevelSetsOf(model.Nodes[v.ID])[2]);
            }

            ++iteration;
        }
    }
}
