using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.FriesHybridCrack
{
    public class CrackBody3DObserver : ICrackObserver
    {
        private readonly CrackSurface3D crack;
        private readonly string outputDirectory;
        private int iteration;

        public CrackBody3DObserver(CrackSurface3D crack, string outputDirectory)
        {
            this.crack = crack;
            this.outputDirectory = outputDirectory;
            iteration = 0;
        }

        public void Update()
        {
            string pathMesh = $"{outputDirectory}\\crack_surface_{crack.ID}_t{iteration}.vtk";
            using (var writer = new VtkFileWriter(pathMesh))
            {
                var outputMesh = new OutputCrackMesh(crack.Vertices, crack.Cells);
                writer.WriteMesh(outputMesh);
            }
            ++iteration;
        }


    }
}
