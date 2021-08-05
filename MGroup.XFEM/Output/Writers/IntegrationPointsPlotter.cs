using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using System.IO;
using MGroup.XFEM.Output.Vtk;

//TODO: Standardize the writer classes and their input. Also simplify them as much as possible (e.g. plot just points, without 
//      having attached values).
namespace MGroup.XFEM.Output
{
    public class IntegrationPointsPlotter : IModelObserver
    {
        private readonly XModel<IXMultiphaseElement> model;
        private readonly bool plotNormalsAtBoundaryPoints;
        private readonly string outputDirectory;
        private int iteration;

        public IntegrationPointsPlotter(string outputDirectory, XModel<IXMultiphaseElement> model, 
            bool plotNormalsAtBoundaryPoints = false)
        {
            this.outputDirectory = outputDirectory;
            this.model = model;
            this.plotNormalsAtBoundaryPoints = plotNormalsAtBoundaryPoints;
            iteration = 0;
        }

        public void Update()
        {
            PlotBulkIntegrationPoints();
            PlotBoundaryIntegrationPoints();
            if (plotNormalsAtBoundaryPoints)
            {
                PlotNormalsAtBoundaryPoints();
            }

            ++iteration;
        }

        public void PlotBoundaryIntegrationPoints()
        {
            string path = Path.Combine(outputDirectory, $"gauss_points_boundary_t{iteration}.vtk");
            var integrationPoints = new Dictionary<double[], double>();
            foreach (IXMultiphaseElement element in model.Elements)
            {
                foreach (GaussPoint gp in element.BoundaryIntegrationPoints)
                {
                    double[] point = element.Interpolation.TransformNaturalToCartesian(element.Nodes, gp.Coordinates);
                    integrationPoints.Add(point, element.ID);
                }
            }
            using (var writer = new VtkPointWriter(path))
            {
                writer.WriteScalarField("element_ids", integrationPoints);
            }
        }

        public void PlotBulkIntegrationPoints()
        {
            string path = Path.Combine(outputDirectory, $"gauss_points_bulk_t{iteration}.vtk");
            var integrationPoints = new Dictionary<double[], double>();
            foreach (IXMultiphaseElement element in model.Elements)
            {
                foreach (GaussPoint gp in element.BulkIntegrationPoints)
                {
                    double[] point = element.Interpolation.TransformNaturalToCartesian(element.Nodes, gp.Coordinates);
                    integrationPoints.Add(point, element.ID);
                }
            }
            using (var writer = new VtkPointWriter(path))
            {
                writer.WriteScalarField("element_ids", integrationPoints);
            }
        }

        public void PlotNormalsAtBoundaryPoints()
        {
            string path = Path.Combine(outputDirectory, $"gauss_points_boundary_normals_t{iteration}.vtk");
            var normalVectors = new Dictionary<double[], double[]>();
            foreach (IXMultiphaseElement element in model.Elements)
            {
                IReadOnlyList<GaussPoint> boundaryPoints = element.BoundaryIntegrationPoints;
                IReadOnlyList<double[]> boundaryPointNormals = element.BoundaryIntegrationPointNormals;

                for (int i = 0; i < boundaryPoints.Count; ++i)
                {
                    double[] point = element.Interpolation.TransformNaturalToCartesian(
                        element.Nodes, boundaryPoints[i].Coordinates);
                    double[] normal = boundaryPointNormals[i];
                    normalVectors.Add(point, normal);
                }
            }
            using (var writer = new VtkPointWriter(path))
            {
                writer.WriteVectorField("normal_vectors", normalVectors);
            }
        }
    }
}
