using System.Collections.Generic;
using System.IO;

using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.XFEM.Output;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Phases;
using MGroup.XFEM.Tests.Utilities;

using Xunit;

namespace MGroup.XFEM.Tests.MultiphaseThermal
{
	public static class LsmBalls3DTests
	{
		private static readonly string outputDirectory = Path.Combine(
			Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName, "Resources", "lsm_balls_3D_temp");
		private static readonly string expectedDirectory = Path.Combine(
			Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName, "Resources", "lsm_balls_3D");

		private static readonly double[] minCoords = { -1.0, -1.0, -1.0 };
		private static readonly double[] maxCoords = { +1.0, +1.0, +1.0 };
		private static readonly int[] numElements = { 20, 20, 20 };
		private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;
		private static readonly int[] numBalls = { 2, 1, 1 };
		private const double ballRadius = 0.3;

		private const int defaultPhaseID = 0;

		private const double conductMatrix = 1E0, conductInclusion = 1E5;
		private const double conductBoundaryMatrixInclusion = 1E1, conductBoundaryInclusionInclusion = 1E2;
		private const double specialHeatCoeff = 1.0;

		[Theory]
		[InlineData(true)]
		public static void TestModel(bool cartesianMesh)
		{
			try
			{
				if (!Directory.Exists(outputDirectory))
				{
					Directory.CreateDirectory(outputDirectory);
				}

				// Create model and LSM
				XModel<IXMultiphaseElement> model = CreateModel(cartesianMesh);
				model.FindConformingSubcells = true;
				PhaseGeometryModel geometryModel = CreatePhases(model, cartesianMesh);

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
				model.GeometryModel.Enricher.Observers.Add(new PhaseEnrichmentPlotter(outputDirectory, model, elementSize, 3));

				// Initialize model state so that everything described above can be tracked
				model.Initialize();

				// Compare output
				var computedFiles = new List<string>();
				computedFiles.Add(Path.Combine(outputDirectory, "level_set1_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "level_set2_t0.vtk"));
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
				expectedFiles.Add(Path.Combine(expectedDirectory, "level_set2_t0.vtk"));
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

		[Fact]
		public static void TestSolution()
		{
			try
			{
				if (!Directory.Exists(outputDirectory))
				{
					Directory.CreateDirectory(outputDirectory);
				}

				// Create model and LSM
				XModel<IXMultiphaseElement> model = CreateModel(true);
				model.FindConformingSubcells = true;
				PhaseGeometryModel geometryModel = CreatePhases(model, true);

				// Run analysis
				model.Initialize();
				(IAlgebraicModel algebraicModel, ISolver solver) = SolverChoice.Skyline.Create(model);
				IGlobalVector solution = Utilities.Analysis.RunThermalStaticAnalysis(model, algebraicModel, solver);

				// Plot temperature and heat flux
				var computedFiles = new List<string>();
				computedFiles.Add(Path.Combine(outputDirectory, "temperature_nodes_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "temperature_gauss_points_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "temperature_field_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "heat_flux_gauss_points_t0.vtk"));
				Plotting.PlotTemperatureAndHeatFlux(model, algebraicModel, solution,
					computedFiles[0], computedFiles[1], computedFiles[2], computedFiles[3]);

				// Compare output
				var expectedFiles = new List<string>();
				expectedFiles.Add(Path.Combine(expectedDirectory, "temperature_nodes_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "temperature_gauss_points_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "temperature_field_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "heat_flux_gauss_points_t0.vtk"));

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

		private static XModel<IXMultiphaseElement> CreateModel(bool cartesianMesh)
		{
			// Materials
			var matrixMaterial = new ThermalMaterial(conductMatrix, specialHeatCoeff);
			var inclusionMaterial = new ThermalMaterial(conductInclusion, specialHeatCoeff);
			var materialField = new MatrixInclusionsThermalMaterialField(matrixMaterial, inclusionMaterial,
				conductBoundaryMatrixInclusion, conductBoundaryInclusionInclusion, defaultPhaseID);

			var model = Models.CreateHexa8Model(minCoords, maxCoords, numElements, cartesianMesh,
				bulkIntegrationOrder, boundaryIntegrationOrder, materialField, true);
			Models.ApplyBCsTemperatureDiffAlongX(model, +100, -100);
			return model;
		}

		private static PhaseGeometryModel CreatePhases(XModel<IXMultiphaseElement> model, bool cartesianMesh)
		{
			List<ISurface3D> balls = Utilities.Phases.CreateBallsStructured3D(minCoords, maxCoords, numBalls, ballRadius, 1.0);
			PhaseGeometryModel geometryModel = Utilities.Phases.CreateLsmPhases3D(model, balls, cartesianMesh);
			geometryModel.Enricher = NodeEnricherMultiphaseNoJunctions.CreateThermalStep(geometryModel);
			return geometryModel;
		}
	}
}
