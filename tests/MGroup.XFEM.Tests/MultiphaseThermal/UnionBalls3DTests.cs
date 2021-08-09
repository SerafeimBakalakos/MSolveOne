using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Phases;
using MGroup.XFEM.Output;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Tests.Utilities;
using Xunit;
using MGroup.XFEM.Materials.Duplicates;
using System.Linq;

namespace MGroup.XFEM.Tests.MultiphaseThermal
{
    public static class UnionBalls3DTests
    {
        private static readonly string outputDirectory = Path.Combine(
            Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName, "Resources", "union_balls_3D_temp");
        private static readonly string expectedDirectory = Path.Combine(
            Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName, "Resources", "union_balls_3D");

        private static readonly double[] minCoords = { -1.0, -1.0, -1.0 };
        private static readonly double[] maxCoords = { +1.0, +1.0, +1.0 };
        private static readonly int[] numElements = { 20, 20, 20 };
        private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;

        private const int defaultPhaseID = 0;

        private const double conductMatrix = 1E0, conductInclusion = 1E5;
        private const double conductBoundaryMatrixInclusion = 1E1, conductBoundaryInclusionInclusion = 1E2;
        private const double specialHeatCoeff = 1.0;

        [Fact]
        public static void TestModel()
        {
            try
            {
                if (!Directory.Exists(outputDirectory))
                {
                    Directory.CreateDirectory(outputDirectory);
                }

                // Create model and LSM
                XModel<IXMultiphaseElement> model = CreateModel();
                model.FindConformingSubcells = true;
                PhaseGeometryModel geometryModel = CreatePhases(model);

                // Plot level sets
                geometryModel.GeometryObservers.Add(new PhaseLevelSetPlotter(outputDirectory, model, geometryModel));

                // Plot phases of nodes
                geometryModel.InteractionObservers.Add(new NodalPhasesPlotter(outputDirectory, model));

                // Plot element - phase boundaries interactions
                geometryModel.InteractionObservers.Add(new LsmElementIntersectionsPlotter(outputDirectory, model));

                // Plot element subcells
                model.ModelObservers.Add(new ConformingMeshPlotter(outputDirectory, model));

                // Plot phases of each element subcell
                model.ModelObservers.Add(new ElementPhasePlotter(outputDirectory, model, geometryModel, defaultPhaseID));

                // Write the size of each phase
                model.ModelObservers.Add(new PhasesSizeWriter(outputDirectory, model, geometryModel));

                // Plot bulk and boundary integration points of each element
                model.ModelObservers.Add(new IntegrationPointsPlotter(outputDirectory, model));

                // Plot enrichments
                double elementSize = (maxCoords[0] - minCoords[0]) / numElements[0];
                model.RegisterEnrichmentObserver(new PhaseEnrichmentPlotter(outputDirectory, model, elementSize, 3));

                // Initialize model state so that everything described above can be tracked
                model.Initialize();

                // Compare output
                var computedFiles = new List<string>();
                computedFiles.Add(Path.Combine(outputDirectory, "level_set1_t0.vtk"));
                computedFiles.Add(Path.Combine(outputDirectory, "nodal_phases_t0.vtk"));
                computedFiles.Add(Path.Combine(outputDirectory, "intersections_t0.vtk"));
                computedFiles.Add(Path.Combine(outputDirectory, "conforming_mesh_t0.vtk"));
                computedFiles.Add(Path.Combine(outputDirectory, "element_phases_t0.vtk"));
                computedFiles.Add(Path.Combine(outputDirectory, "phase_sizes_t0.txt"));
                computedFiles.Add(Path.Combine(outputDirectory, "gauss_points_bulk_t0.vtk"));
                computedFiles.Add(Path.Combine(outputDirectory, "gauss_points_boundary_t0.vtk"));
                computedFiles.Add(Path.Combine(outputDirectory, "enriched_nodes_heaviside_t0.vtk"));

                var expectedFiles = new List<string>();
                expectedFiles.Add(Path.Combine(expectedDirectory, "level_set1_t0.vtk"));
                expectedFiles.Add(Path.Combine(expectedDirectory, "nodal_phases_t0.vtk"));
                expectedFiles.Add(Path.Combine(expectedDirectory, "intersections_t0.vtk"));
                expectedFiles.Add(Path.Combine(expectedDirectory, "conforming_mesh_t0.vtk"));
                expectedFiles.Add(Path.Combine(expectedDirectory, "element_phases_t0.vtk"));
                expectedFiles.Add(Path.Combine(expectedDirectory, "phase_sizes_t0.txt"));
                expectedFiles.Add(Path.Combine(expectedDirectory, "gauss_points_bulk_t0.vtk"));
                expectedFiles.Add(Path.Combine(expectedDirectory, "gauss_points_boundary_t0.vtk"));
                expectedFiles.Add(Path.Combine(expectedDirectory, "enriched_nodes_heaviside_t0.vtk"));

                double tolerance = 1E-6;
                for (int i = 0; i < expectedFiles.Count; ++i)
                {
                    Assert.True(IOUtilities.AreDoubleValueFilesEquivalent(expectedFiles[i], computedFiles[i], tolerance));
                }
            }
            finally
            {
                if (Directory.Exists(outputDirectory))
                {
                    DirectoryInfo di = new DirectoryInfo(outputDirectory);
                    di.Delete(true);//true means delete subdirectories and files
                }
            }
        }

        private static XModel<IXMultiphaseElement> CreateModel()
        {
            // Materials
            var matrixMaterial = new ThermalMaterial(conductMatrix, specialHeatCoeff);
            var inclusionMaterial = new ThermalMaterial(conductInclusion, specialHeatCoeff);
            var materialField = new MatrixInclusionsThermalMaterialField(matrixMaterial, inclusionMaterial,
                conductBoundaryMatrixInclusion, conductBoundaryInclusionInclusion, defaultPhaseID);

            var model = Models.CreateHexa8Model(minCoords, maxCoords, numElements, true,
                bulkIntegrationOrder, boundaryIntegrationOrder, materialField, true);
            Models.ApplyBCsTemperatureDiffAlongX(model, 100, -100);
            return model;
        }

        private static PhaseGeometryModel CreatePhases(XModel<IXMultiphaseElement> model)
        {
            var geometricModel = new PhaseGeometryModel(model);
            model.GeometryModel = geometricModel;
            geometricModel.Enricher = NodeEnricherMultiphaseNoJunctions.CreateThermalStep(geometricModel);
            geometricModel.MergeOverlappingPhases = true;
            var defaultPhase = new DefaultPhase();
            geometricModel.Phases[defaultPhase.ID] = defaultPhase;

            var balls = new Sphere[2];
            balls[0] = new Sphere(-0.25, 0, 0, 0.5);
            balls[1] = new Sphere(+0.25, 0, 0, 0.4);
            for (int p = 0; p < balls.Length; ++p)
            {
                var phase = new LsmPhase(p + 1, geometricModel, 0);
                var curve = new SimpleLsm3D(phase.ID, model.Nodes.Values.OrderBy(n => n.ID).ToList(), balls[p]);
                geometricModel.Phases[phase.ID] = phase;

                var boundary = new ClosedPhaseBoundary(phase.ID, curve, defaultPhase, phase);
                defaultPhase.ExternalBoundaries.Add(boundary);
                defaultPhase.Neighbors.Add(phase);
                phase.ExternalBoundaries.Add(boundary);
                phase.Neighbors.Add(defaultPhase);
                geometricModel.PhaseBoundaries[boundary.ID] = boundary;
            }
            return geometricModel;
        }
    }
}
