using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.FriesHybridCrack
{
    public class CrackBody2DObserver : ICrackObserver
    {
        private readonly CrackCurve2D crack;
        private readonly string outputDirectory;
        private int iteration;

        public CrackBody2DObserver(CrackCurve2D crack, string outputDirectory)
        {
            this.crack = crack;
            this.outputDirectory = outputDirectory;
            iteration = 0;
        }

        public void Update()
        {
            string pathMesh = $"{outputDirectory}\\crack_curve_{crack.ID}_t{iteration}.vtk";
            using (var writer = new VtkFileWriter(pathMesh))
            {
                var outputMesh = new OutputCrackMesh(crack.Vertices, crack.Cells);
                writer.WriteMesh(outputMesh);
            }
            ++iteration;
        }


    }
}
