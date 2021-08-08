//This example is already covered in BallInclusions2DTests
//using System;
//using System.Collections.Generic;
//using System.IO;
//using System.Linq;
//using System.Text;
//using ISAAR.MSolve.Discretization;
//using ISAAR.MSolve.Discretization.FreedomDegrees;
//using ISAAR.MSolve.LinearAlgebra.Matrices;
//using ISAAR.MSolve.LinearAlgebra.Vectors;
//using ISAAR.MSolve.Materials;
//using MGroup.XFEM.Elements;
//using MGroup.XFEM.Enrichment.Enrichers;
//using MGroup.XFEM.Enrichment.SingularityResolution;
//using MGroup.XFEM.Entities;
//using MGroup.XFEM.Geometry.LSM;
//using MGroup.XFEM.Geometry.Primitives;
//using MGroup.XFEM.Materials;
//using MGroup.XFEM.Output;
//using MGroup.XFEM.Output.Writers;
//using MGroup.XFEM.Phases;
//using MGroup.XFEM.Tests.Utilities;
//using Xunit;

//namespace MGroup.XFEM.Tests.MultiphaseStructural
//{
//    public static class CohesiveInclusions2DTests
//    {
//        private static readonly string outputDirectory = Path.Combine(
//            Directory.GetParent(Directory.GetCurrentDirectory()).Parent.Parent.FullName, "Resources", "cohesive_inclusions_2D_temp");
//        private static readonly string expectedDirectory = Path.Combine(
//            Directory.GetParent(Directory.GetCurrentDirectory()).Parent.Parent.FullName, "Resources", "cohesive_inclusions_2D");

//        private static readonly double[] minCoords = { -1.0, -1.0 };
//        private static readonly double[] maxCoords = { +1.0, +1.0 };
//        private const double thickness = 1.0;
//        private static readonly int[] numElements = { 31, 31 };
//        private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;

//        private const double matrixE = 1, inclusionE = 2, v = 0.3;
//        //private const double cohesivenessNormal = 0, cohesivenessTangent = 0;
//        private const double cohesivenessNormal = 1000, cohesivenessTangent = 1000;

//        private static readonly int[] numBalls = { 2, 1 };
//        private const double ballRadius = 0.25;
//        private const int defaultPhaseID = 0;


//        //[Fact]
//        public static void TestModel()
//        {
//            try
//            {
//                if (!Directory.Exists(outputDirectory))
//                {
//                    Directory.CreateDirectory(outputDirectory);
//                }

//                // Create model and LSM
//                XModel<IXMultiphaseElement> model = CreateModel();
//                model.FindConformingSubcells = true;
//                PhaseGeometryModel geometryModel = CreatePhases(model);

//                // Plot level sets
//                geometryModel.GeometryObservers.Add(new PhaseLevelSetPlotter(outputDirectory, model, geometryModel));

//                // Plot phases of nodes
//                geometryModel.InteractionObservers.Add(new NodalPhasesPlotter(outputDirectory, model));

//                // Plot element - phase boundaries interactions
//                geometryModel.InteractionObservers.Add(new LsmElementIntersectionsPlotter(outputDirectory, model));

//                // Plot element subcells
//                model.ModelObservers.Add(new ConformingMeshPlotter(outputDirectory, model));

//                // Plot phases of each element subcell
//                model.ModelObservers.Add(new ElementPhasePlotter(outputDirectory, model, geometryModel, defaultPhaseID));


//                // Plot bulk and boundary integration points of each element
//                model.ModelObservers.Add(new IntegrationPointsPlotter(outputDirectory, model, true));

//                // Plot enrichments
//                double elementSize = (maxCoords[0] - minCoords[0]) / numElements[0];
//                model.RegisterEnrichmentObserver(new PhaseEnrichmentPlotter(outputDirectory, model, elementSize, 2));

//                // Initialize model state so that everything described above can be tracked
//                model.Initialize();

//                // Compare output
//                var computedFiles = new List<string>();
//                computedFiles.Add(Path.Combine(outputDirectory, "level_set1_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "level_set2_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "nodal_phases_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "intersections_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "conforming_mesh_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "element_phases_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "phase_sizes_t0.txt"));
//                computedFiles.Add(Path.Combine(outputDirectory, "gauss_points_bulk_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "gauss_points_boundary_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "gauss_points_boundary_normals_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "enriched_nodes_heaviside_t0.vtk"));

//                var expectedFiles = new List<string>();
//                expectedFiles.Add(Path.Combine(expectedDirectory, "level_set1_t0.vtk"));
//                expectedFiles.Add(Path.Combine(expectedDirectory, "level_set2_t0.vtk"));
//                expectedFiles.Add(Path.Combine(expectedDirectory, "nodal_phases_t0.vtk"));
//                expectedFiles.Add(Path.Combine(expectedDirectory, "intersections_t0.vtk"));
//                expectedFiles.Add(Path.Combine(expectedDirectory, "conforming_mesh_t0.vtk"));
//                expectedFiles.Add(Path.Combine(expectedDirectory, "element_phases_t0.vtk"));
//                expectedFiles.Add(Path.Combine(expectedDirectory, "phase_sizes_t0.txt"));
//                expectedFiles.Add(Path.Combine(expectedDirectory, "gauss_points_bulk_t0.vtk"));
//                expectedFiles.Add(Path.Combine(expectedDirectory, "gauss_points_boundary_t0.vtk"));
//                expectedFiles.Add(Path.Combine(expectedDirectory, "gauss_points_boundary_normals_t0.vtk"));
//                expectedFiles.Add(Path.Combine(expectedDirectory, "enriched_nodes_heaviside_t0.vtk"));

//                double tolerance = 1E-6;
//                for (int i = 0; i < expectedFiles.Count; ++i)
//                {
//                    Assert.True(IOUtilities.AreDoubleValueFilesEquivalent(expectedFiles[i], computedFiles[i], tolerance));
//                }
//            }
//            finally
//            {
//                if (Directory.Exists(outputDirectory))
//                {
//                    DirectoryInfo di = new DirectoryInfo(outputDirectory);
//                    di.Delete(true);//true means delete subdirectories and files
//                }
//            }
//        }

//        //[Fact]
//        public static void TestSolution()
//        {
//            try
//            {
//                if (!Directory.Exists(outputDirectory))
//                {
//                    Directory.CreateDirectory(outputDirectory);
//                }

//                // Create model and LSM
//                XModel<IXMultiphaseElement> model = CreateModel();
//                model.FindConformingSubcells = true;
//                PhaseGeometryModel geometryModel = CreatePhases(model);

//                // Run analysis
//                model.Initialize();
//                IVectorView solution = Analysis.RunStructuralStaticAnalysis(model);

//                // Plot displacements, strains, stresses
//                var computedFiles = new List<string>();
//                computedFiles.Add(Path.Combine(outputDirectory, "displacement_nodes_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "displacement_gauss_points_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "strains_gauss_points_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "stresses_gauss_points_t0.vtk"));
//                computedFiles.Add(Path.Combine(outputDirectory, "displacement_strain_stress_field_t0.vtk"));
//                Utilities.Plotting.PlotDisplacements(model, solution, computedFiles[0], computedFiles[1]);
//                Utilities.Plotting.PlotStrainsStressesAtGaussPoints(model, solution, computedFiles[2], computedFiles[3]);
//                Utilities.Plotting.PlotDisplacementStrainStressFields(model, solution, computedFiles[4]);

//                // Compare output
//                var expectedFiles = new List<string>();
//                //expectedFiles.Add(Path.Combine(expectedDirectory, "temperature_nodes_t0.vtk"));
//                //expectedFiles.Add(Path.Combine(expectedDirectory, "temperature_gauss_points_t0.vtk"));
//                //expectedFiles.Add(Path.Combine(expectedDirectory, "temperature_field_t0.vtk"));
//                //expectedFiles.Add(Path.Combine(expectedDirectory, "heat_flux_gauss_points_t0.vtk"));

//                double tolerance = 1E-6;
//                for (int i = 0; i < expectedFiles.Count; ++i)
//                {
//                    Assert.True(IOUtilities.AreDoubleValueFilesEquivalent(expectedFiles[i], computedFiles[i], tolerance));
//                }
//            }
//            finally
//            {
//                if (Directory.Exists(outputDirectory))
//                {
//                    DirectoryInfo di = new DirectoryInfo(outputDirectory);
//                    di.Delete(true);//true means delete subdirectories and files
//                }
//            }
//        }

//        private static XModel<IXMultiphaseElement> CreateModel()
//        {
//            // Materials
//            var materialMatrix = new ElasticMaterial2D(StressState2D.PlaneStress) { YoungModulus = matrixE, PoissonRatio = v };
//            var materialInclusion = new ElasticMaterial2D(StressState2D.PlaneStress) { YoungModulus = inclusionE, PoissonRatio = v };
//            var interfaceMaterial = new CohesiveInterfaceMaterial(Matrix.CreateFromArray(new double[,]
//            {
//                { cohesivenessNormal, 0 },
//                { 0, cohesivenessTangent }
//            }));
//            var materialField = new MatrixInclusionsStructuralMaterialField(
//                materialMatrix, materialInclusion, interfaceMaterial, 0);

//            // Setup model
//            XModel<IXMultiphaseElement> model = Models.CreateQuad4Model(minCoords, maxCoords, thickness, numElements,
//                bulkIntegrationOrder, boundaryIntegrationOrder, materialField, true);
//            Models.ApplyBCsCantileverTension(model);

//            return model;
//        }

//        private static PhaseGeometryModel CreatePhases(XModel<IXMultiphaseElement> model)
//        {
//            var geometricModel = new PhaseGeometryModel(model);
//            model.GeometryModel = geometricModel;
//            geometricModel.Enricher = NodeEnricherMultiphaseNoJunctions.CreateStructuralStep(geometricModel, 2);
//            List<SimpleLsm2D> lsmCurves = InitializeLSM(model);
//            var defaultPhase = new DefaultPhase();
//            geometricModel.Phases[defaultPhase.ID] = defaultPhase;
//            for (int p = 0; p < lsmCurves.Count; ++p)
//            {
//                SimpleLsm2D curve = lsmCurves[p];
//                var phase = new LsmPhase(p + 1, geometricModel, -1);
//                geometricModel.Phases[phase.ID] = phase;

//                var boundary = new ClosedPhaseBoundary(phase.ID, curve, defaultPhase, phase);
//                defaultPhase.ExternalBoundaries.Add(boundary);
//                defaultPhase.Neighbors.Add(phase);
//                phase.ExternalBoundaries.Add(boundary);
//                phase.Neighbors.Add(defaultPhase);
//                geometricModel.PhaseBoundaries[boundary.ID] = boundary;
//            }
//            return geometricModel;
//        }

//        private static List<SimpleLsm2D> InitializeLSM(XModel<IXMultiphaseElement> model)
//        {
//            double xMin = minCoords[0], xMax = maxCoords[0], yMin = minCoords[1], yMax = maxCoords[1];
//            var curves = new List<SimpleLsm2D>(numBalls[0] * numBalls[1]);
//            double dx = (xMax - xMin) / (numBalls[0] + 1);
//            double dy = (yMax - yMin) / (numBalls[1] + 1);
//            int id = 1;
//            for (int i = 0; i < numBalls[0]; ++i)
//            {
//                double centerX = xMin + (i + 1) * dx;
//                for (int j = 0; j < numBalls[1]; ++j)
//                {
//                    double centerY = yMin + (j + 1) * dy;
//                    var circle = new Circle2D(centerX, centerY, ballRadius);
//                    var lsm = new SimpleLsm2D(id++, model.XNodes, circle);
//                    curves.Add(lsm);
//                }
//            }

//            return curves;
//        }
//    }
//}
