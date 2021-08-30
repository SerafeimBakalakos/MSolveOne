using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Output.Vtk;

//TODO: Duplicate code: almost identical to CrackSurfaceBody3DObserver. Use a base class CrackSurface3DObserver and template method.
namespace MGroup.XFEM.Output.FriesHybridCrack
{
    public class CrackExtension3DObserver : ICrackObserver
    {
        private readonly CrackSurface3D crack;
        private readonly string outputDirectory;
        private int iteration;

        public CrackExtension3DObserver(CrackSurface3D crack, string outputDirectory)
        {
            this.crack = crack;
            this.outputDirectory = outputDirectory;
            iteration = 0;
        }

        public void Update()
        {
            string pathMesh = $"{outputDirectory}\\crack_extension_{crack.ID}_t{iteration}.vtk";
            using (var writer = new VtkFileWriter(pathMesh))
            {
                List<Vertex3D> vertices = crack.CrackFront.Vertices.Concat(crack.CrackExtension.ExtensionVertices).ToList();
                var outputMesh = new OutputCrackMesh(vertices, crack.CrackExtension.Cells);
                writer.WriteMesh(outputMesh);
            }

            ++iteration;
        }


    }
}
