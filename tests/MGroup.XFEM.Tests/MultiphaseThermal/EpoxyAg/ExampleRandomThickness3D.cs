using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.XFEM.Output;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Tests.Utilities;

namespace MGroup.XFEM.Tests.MultiphaseThermal.EpoxyAg
{
	public static class ExampleRandomThickness3D
	{
		private static readonly string outputDirectory = @"C:\Users\Serafeim\Desktop\HEAT\2020\EpoxyAG\UniformThickness3D\";

		private static readonly double[] minCoords = { -2000.0, -2000.0, -2000.0 };
		private static readonly double[] maxCoords = { +2000.0, +2000.0, +2000.0 };
		private static readonly int[] numElements = { 50, 50, 50 };
		private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;

		private const int defaultPhaseID = 0;

		//private const int numBalls = 8, rngSeed = 33; //problems in intersection mesh
		//private const int numBalls = 8, rngSeed = 13;//problems in intersection mesh
		//private const int numBalls = 8, rngSeed = 17;
		private const int numBalls = 100, rngSeed = 33;
		//private const double epoxyPhaseRadius = 0.2, silverPhaseThickness = 0.1;

		private const double conductEpoxy = 0.25, conductSilver = 429;
		private const double conductBoundaryEpoxySilver = conductEpoxy;
		private const double specialHeatCoeff = 1.0;

		public static void RunModelCreation()
		{
			// Create model and LSM
			(XModel<IXMultiphaseElement> model, ThermalBiMaterialField materialField) = CreateModel();
			model.FindConformingSubcells = true;
			GeometryPreprocessor3DRandomThickness geometryPreprocessor = CreatePhases(model, materialField);
			var geometryModel = geometryPreprocessor.GeometryModel;

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

			// Plot bulk and boundary integration points of each element
			model.ModelObservers.Add(new IntegrationPointsPlotter(outputDirectory, model));

			// Plot enrichments
			double elementSize = (maxCoords[0] - minCoords[0]) / numElements[0];
			model.GeometryModel.Enricher.Observers.Add(new PhaseEnrichmentPlotter(outputDirectory, model, elementSize, 3));

			// Initialize model state so that everything described above can be tracked
			model.Initialize();

			Console.WriteLine(geometryPreprocessor.PrintVolumes());
		}

		public static void RunAnalysis()
		{
			if (!Directory.Exists(outputDirectory))
			{
				Directory.CreateDirectory(outputDirectory);
			}

			// Create model and LSM
			(XModel<IXMultiphaseElement> model, ThermalBiMaterialField materialField) = CreateModel();
			model.FindConformingSubcells = true;
			GeometryPreprocessor3DRandomThickness geometryPreprocessor = CreatePhases(model, materialField);

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
		}

		//public static void RunHomogenization()
		//{
		//    // Create physical model, LSM and phases
		//    Console.WriteLine("Creating physical and geometric models");
		//    (XModel<IXMultiphaseElement> model, ThermalBiMaterialField materialField) = CreateModel();
		//    model.FindConformingSubcells = true;
		//    GeometryPreprocessor3DRandomThickness geometryPreprocessor = CreatePhases(model, materialField);

		//    // Geometric interactions
		//    model.Initialize();

		//    // Write volume fractions
		//    Console.WriteLine(geometryPreprocessor.PrintVolumes());

		//    // Run homogenization analysis
		//    IMatrix conductivity = Analysis.RunHomogenizationAnalysisThermal3D(model, minCoords, maxCoords);
		//    Console.WriteLine(
		//        $"conductivity = [ {conductivity[0, 0]} {conductivity[0, 1]} {conductivity[0, 2]};"
		//        + $" {conductivity[1, 0]} {conductivity[1, 1]} {conductivity[1, 2]};"
		//        + $" {conductivity[2, 0]} {conductivity[2, 1]} {conductivity[2, 2]} ]");
		//}

		private static (XModel<IXMultiphaseElement>, ThermalBiMaterialField) CreateModel()
		{
			// Materials
			var epoxyMaterial = new ThermalMaterial(conductEpoxy, specialHeatCoeff);
			var silverMaterial = new ThermalMaterial(conductSilver, specialHeatCoeff);
			var materialField = new ThermalBiMaterialField(epoxyMaterial, silverMaterial, conductBoundaryEpoxySilver);

			var model = Models.CreateHexa8Model(minCoords, maxCoords, numElements, true,
				bulkIntegrationOrder, boundaryIntegrationOrder, materialField, true);
			Models.ApplyBCsTemperatureDiffAlongX(model, +100, -100);
			return (model, materialField);
		}

		private static GeometryPreprocessor3DRandomThickness CreatePhases(
			XModel<IXMultiphaseElement> model, ThermalBiMaterialField materialField)
		{
			var preprocessor = new GeometryPreprocessor3DRandomThickness(model);
			preprocessor.MinCoordinates = minCoords;
			preprocessor.MaxCoordinates = maxCoords;
			preprocessor.NumBalls = numBalls;
			preprocessor.RngSeed = rngSeed;

			preprocessor.GeneratePhases(model);
			materialField.PhasesWithMaterial0.Add(preprocessor.MatrixPhaseID);
			foreach (int p in preprocessor.EpoxyPhaseIDs) materialField.PhasesWithMaterial0.Add(p);
			foreach (int p in preprocessor.SilverPhaseIDs) materialField.PhasesWithMaterial1.Add(p);

			return preprocessor;
		}
	}
}
