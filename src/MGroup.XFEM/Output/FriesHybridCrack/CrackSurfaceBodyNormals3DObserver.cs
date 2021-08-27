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
    public class CrackSurfaceBodyNormals3DObserver : ICrackObserver
    {
        private readonly CrackSurface3D crack;
        private readonly string outputDirectory;
        private readonly bool plotPseudoNormals;
        private int iteration;

        public CrackSurfaceBodyNormals3DObserver(CrackSurface3D crack, string outputDirectory, bool plotPseudoNormals)
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
                string pathVertexNormals = $"{outputDirectory}\\crack_surface_normals_vertices_{crack.ID}_t{iteration}.vtk";
                using (var writer = new VtkPointWriter(pathVertexNormals))
                {
                    var normalVectors = new Dictionary<double[], double[]>();
                    foreach (Vertex3D vertex in crack.Vertices)
                    {
                        normalVectors[vertex.CoordsGlobal] = vertex.PseudoNormal;
                    }

                    writer.WriteVectorField("normals", normalVectors);
                }

                // Normals of edges
                string pathEdgeNormals = $"{outputDirectory}\\crack_surface_normals_edges_{crack.ID}_t{iteration}.vtk";
                using (var writer = new VtkPointWriter(pathEdgeNormals))
                {
                    var normalVectors = new Dictionary<double[], double[]>();
                    foreach (Edge3D edge in crack.Edges)
                    {
                        double[] start = edge.Start.CoordsGlobal;
                        double[] end = edge.End.CoordsGlobal;
                        double[] middle = new double[start.Length];
                        for (int d = 0; d < start.Length; ++d)
                        {
                            middle[d] = 0.5 * (start[d] + end[d]);
                        }
                        normalVectors[middle] = edge.PseudoNormal;
                    }

                    writer.WriteVectorField("normals", normalVectors);
                }
            }

            // Normals of cells
            string pathCellNormals = $"{outputDirectory}\\crack_surface_normals_cells_{crack.ID}_t{iteration}.vtk";
            using (var writer = new VtkPointWriter(pathCellNormals))
            {
                var normalVectors = new Dictionary<double[], double[]>();
                foreach (TriangleCell3D cell in crack.Cells)
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
