using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.FriesHybridCrack
{
    public class CrackExtensionNormals2DObserver : ICrackObserver
    {
        private readonly CrackCurve2D crack;
        private readonly string outputDirectory;
        private readonly bool plotPseudoNormals;
        private int iteration;

        public CrackExtensionNormals2DObserver(CrackCurve2D crack, string outputDirectory, bool plotPseudoNormals)
        {
            this.crack = crack;
            this.outputDirectory = outputDirectory;
            this.plotPseudoNormals = plotPseudoNormals;
            iteration = 0;
        }

        public void Update()
        {
            if (plotPseudoNormals)
            {
                // Normals of vertices
                string pathVertexNormals = $"{outputDirectory}\\crack_extension_normals_vertices_{crack.ID}_t{iteration}.vtk";
                using (var writer = new VtkPointWriter(pathVertexNormals))
                {
                    var normalVectors = new Dictionary<double[], double[]>();
                    foreach (Vertex2D vertex in crack.CrackExtension.ExtensionVertices)
                    {
                        normalVectors[vertex.CoordsGlobal] = vertex.PseudoNormal;
                    }

                    writer.WriteVectorField("normals", normalVectors);
                }
            }

            // Normals of cells
            string pathCellNormals = $"{outputDirectory}\\crack_extension_normals_cells_{crack.ID}_t{iteration}.vtk";
            using (var writer = new VtkPointWriter(pathCellNormals))
            {
                var normalVectors = new Dictionary<double[], double[]>();
                foreach (LineCell2D cell in crack.CrackExtension.Cells)
                {
                    IList<double[]> vertices = cell.Vertices.Select(v => v.CoordsGlobal).ToList();
                    double[] centroid = Utilities.FindCentroid(vertices);
                    normalVectors[centroid] = cell.Normal; //TODO: Perhaps I should scale it by the area.
                }

                writer.WriteVectorField("normals", normalVectors);
            }

            ++iteration;
        }
    }
}
