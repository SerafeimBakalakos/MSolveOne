using System;
using System.Collections.Generic;
using System.IO;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Meshes.Structured;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.XFEM.Output.Fields;
using MGroup.XFEM.Output.Vtk;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Phases;
using MGroup.XFEM.Tests.Utilities;

using Xunit;

namespace MGroup.XFEM.Tests.MultiphaseThermal.PINN
{
	public static class PinnThermalTests
	{
		private static readonly string outputDirectory = @"C:\Users\Serafeim\Desktop\HEAT\PINN\thermal_example_3d";

		private static readonly double[] minCoords = { 0.0, 0.0, 0.0 };
		private static readonly double[] maxCoords = { 1E0, 1E0, 1E0 }; //μm
		private static readonly int[] numElementsCoarse = { 5, 5, 5 };
		private static readonly int[] numElementsFine = { 120, 120, 120 };
		private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;
		
		private const double ballRadius = 0.25; // μm
		private static readonly int[] numBalls = { 1, 1, 1 };

		private const int defaultPhaseID = 0;

		private const bool cohesiveInterfaces = true;
		private const double conductMatrix = 0.1, conductInclusion = 1; //Wμ/K
		private const double conductBoundaryMatrixInclusion = 1E0, conductBoundaryInclusionInclusion = 0;
		private const double specialHeatCoeff = 1.0;

		[Fact]
		public static void TestModel()
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
			//geometryModel.GeometryObservers.Add(new PhaseLevelSetPlotter(outputDirectory, model, geometryModel));

			// Plot phases of nodes
			//geometryModel.InteractionObservers.Add(new NodalPhasesPlotter(outputDirectory, model));

			// Plot element - phase boundaries interactions
			geometryModel.InteractionObservers.Add(new LsmElementIntersectionsPlotter(outputDirectory, model));

			// Plot element subcells
			model.ModelObservers.Add(new ConformingMeshPlotter(outputDirectory, model));

			// Plot phases of each element subcell
			model.ModelObservers.Add(new ElementPhasePlotter(outputDirectory, model, geometryModel, defaultPhaseID));

			// Write the size of each phase
			//model.ModelObservers.Add(new PhasesSizeWriter(outputDirectory, model, geometryModel));

			// Plot bulk and boundary integration points of each element
			//model.ModelObservers.Add(new IntegrationPointsPlotter(outputDirectory, model));

			// Plot enrichments
			//double elementSize = (maxCoords[0] - minCoords[0]) / numElements[0];
			//model.RegisterEnrichmentObserver(new PhaseEnrichmentPlotter(outputDirectory, model, elementSize, 3));

			// Initialize model state so that everything described above can be tracked
			model.Initialize();
		}

		[Fact]
		public static void TestSolution()
		{
			if (!Directory.Exists(outputDirectory))
			{
				Directory.CreateDirectory(outputDirectory);
			}

			// Create model and LSM
			XModel<IXMultiphaseElement> model = CreateModel();
			model.FindConformingSubcells = true;
			PhaseGeometryModel geometryModel = CreatePhases(model);

			// Run analysis
			model.Initialize();
			(IAlgebraicModel algebraicModel, ISolver solver) = SolverChoice.SuiteSparse.Create(model);
			IGlobalVector solution = Utilities.Analysis.RunThermalStaticAnalysis(model, algebraicModel, solver);

			// Plot temperature and heat flux
			PlotTemperatureAtArbitraryPoints(model, algebraicModel, solution);
			//var computedFiles = new List<string>();
			//computedFiles.Add(Path.Combine(outputDirectory, "temperature_nodes_t0.vtk"));
			//computedFiles.Add(Path.Combine(outputDirectory, "temperature_gauss_points_t0.vtk"));
			//computedFiles.Add(Path.Combine(outputDirectory, "temperature_field_t0.vtk"));
			//computedFiles.Add(Path.Combine(outputDirectory, "heat_flux_gauss_points_t0.vtk"));
			//Utilities.Plotting.PlotTemperatureAndHeatFlux(model, algebraicModel, solution,
			//computedFiles[0], computedFiles[1], computedFiles[2], computedFiles[3]);

			//Utilities.Plotting.WriteNodalTemperatures(
			//model, algebraicModel, solution, Path.Combine(outputDirectory, "nodal_temperature.txt"));
		}


		////[Fact]
		//public static void TestHomogenization()
		//{
		//	if (!Directory.Exists(outputDirectory))
		//	{
		//		Directory.CreateDirectory(outputDirectory);
		//	}

		//	// Create model and LSM
		//	XModel<IXMultiphaseElement> model = CreateModel();
		//	model.FindConformingSubcells = true;
		//	PhaseGeometryModel geometryModel = CreatePhases(model);

		//	// Run homogenization analysis
		//	model.Initialize();
		//	IMatrix conductivity = Analysis.RunHomogenizationAnalysisThermal3D(model, minCoords, maxCoords);

		//	// Print results
		//	string pathResults = outputDirectory + "\\equiv_cond.txt";
		//	using (var writer = new StreamWriter(pathResults, true))
		//	{
		//		writer.WriteLine();
		//		writer.WriteLine("#################################################################");
		//		writer.WriteLine("Date = " + DateTime.Now);
		//		writer.WriteLine(
		//			$"conductivity = [ {conductivity[0, 0]} {conductivity[0, 1]} {conductivity[0, 2]};" +
		//			$" {conductivity[1, 0]} {conductivity[1, 1]} {conductivity[1, 2]};" +
		//			$" {conductivity[2, 0]} {conductivity[2, 1]} {conductivity[2, 2]}]");
		//	}
		//}

		private static XModel<IXMultiphaseElement> CreateModel()
		{
			// Materials
			var matrixMaterial = new ThermalMaterial(conductMatrix, specialHeatCoeff);
			var inclusionMaterial = new ThermalMaterial(conductInclusion, specialHeatCoeff);
			var materialField = new MatrixInclusionsThermalMaterialField(matrixMaterial, inclusionMaterial,
				conductBoundaryMatrixInclusion, conductBoundaryInclusionInclusion, defaultPhaseID);

			var model = Models.CreateHexa8Model(minCoords, maxCoords, numElementsCoarse, true,
				bulkIntegrationOrder, boundaryIntegrationOrder, materialField, cohesiveInterfaces);
			Models.ApplyBCsTemperatureDiffAlongX(model, 0, 1);
			return model;
		}

		private static PhaseGeometryModel CreatePhases(XModel<IXMultiphaseElement> model)
		{
			var geometryPreprocessor = new GeometryPreprocessor3D(model);
			geometryPreprocessor.MinCoords = minCoords;
			geometryPreprocessor.MaxCoords = maxCoords;
			geometryPreprocessor.BallRadius = ballRadius;
			geometryPreprocessor.NumBalls = numBalls;
			var dualMesh = new DualCartesianMesh3D.Builder(minCoords, maxCoords, numElementsCoarse, numElementsFine).BuildMesh();
			geometryPreprocessor.GeneratePhases(model, dualMesh);

			PhaseGeometryModel geometricModel = geometryPreprocessor.GeometryModel;
			model.GeometryModel = geometricModel;
			geometricModel.Enricher = NodeEnricherMultiphaseNoJunctions.CreateThermalStep(geometricModel);
			return geometricModel;
		}

		private static void PlotTemperatureAtArbitraryPoints(XModel<IXMultiphaseElement> model, IAlgebraicModel algebraicModel, 
			IGlobalVector solution)
		{
			// Temperature at arbitrary points
			List<double[]> points = GeneratePlotPoints();
			var mesh = new UniformCartesianMesh3D.Builder(minCoords, maxCoords, numElementsCoarse).BuildMesh();
			var postProcessor = new TemperaturePostProcessor(mesh, model, algebraicModel);
			List<double> temperatures = postProcessor.CalcTemperatureAt(points, solution);

			string pathTemperatureArbitraryPoints = Path.Combine(outputDirectory, "temperature_arbitrary_points.vtk");
			using (var writer = new VtkPointWriter(pathTemperatureArbitraryPoints))
			{
				writer.WriteScalarField("temperature", points, temperatures);
			}
		}

		private static List<double[]> GeneratePlotPoints()
		{
			var points = new List<double[]>();

			double epsilon = 5E-4 * (maxCoords[0] - minCoords[0]);
			double[] center =
			{
				0.5 * (maxCoords[0] - minCoords[0]), 0.5 * (maxCoords[1] - minCoords[1]) , 0.5 * (maxCoords[2] - minCoords[2])
			};
			double rInternal = ballRadius - epsilon;
			double rExternal = ballRadius + epsilon;
			int numPointsPerCircle = 20;

			//for (int i = 0; i < numPointsPerCircle; ++i)
			//{
			//    double phi = i * (2 * Math.PI) / numPointsPerCircle; // 0 <= phi < 2*pi
			//    for (int j = 0; j < numPointsPerCircle; ++j)
			//    {
			//        double theta = j * Math.PI / numPointsPerCircle; // 0 <= theta <= pi 

			//        points.Add(new double[]
			//        {
			//            center[0] + rInternal * Math.Cos(phi) * Math.Sin(theta),
			//            center[1] + rInternal * Math.Sin(phi) * Math.Sin(theta),
			//            center[2] + rInternal * Math.Cos(theta)
			//        });

			//        points.Add(new double[]
			//        {
			//            center[0] + rExternal * Math.Cos(phi) * Math.Sin(theta),
			//            center[1] + rExternal * Math.Sin(phi) * Math.Sin(theta),
			//            center[2] + rExternal * Math.Cos(theta)
			//        });
			//    }
			//}

			//points.Add(center);
			points.Add(new double[] { center[0] + ballRadius - epsilon, center[1], center[2] });
			points.Add(new double[] { center[0] + ballRadius + epsilon, center[1], center[2] });
			//points.Add(new double[] { maxCoords[0] - epsilon, center[1], center[2] });
			//points.Add(new double[] { center[0], center[1] + ballRadius - epsilon, center[2] });
			//points.Add(new double[] { center[0], center[1] + ballRadius + epsilon, center[2] });
			//points.Add(new double[] { center[0], maxCoords[1] - epsilon, center[2] });


			return points;
		}
	}
}
