using System;
using System.Collections.Generic;
using System.Diagnostics;
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
using System.Linq;
using MGroup.XFEM.Materials.Duplicates;

namespace MGroup.XFEM.Tests.MultiphaseThermal.EpoxyAg
{
	public class Benchmark3D
	{
		private static readonly string outputDirectory = @"C:\Users\Serafeim\Desktop\HEAT\2020\Benchmark3D";

		private static readonly double[] minCoords = { -700.0, -700.0, -700.0 };
		private static readonly double[] maxCoords = { +700.0, +700.0, +700.0 };
		private static readonly int[] numElements = { 20, 20, 20 };
		private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;

		private const int defaultPhaseID = 0;

		private static readonly int[] numBalls = { 2, 1, 1};
		private const double internalRadius = 320/*, externalRadius = 400*/;

		private const double conductMatrix = 0.25, conductInclusion = 430;
		private const double conductBoundaryMatrixInclusion = 1E10, conductBoundaryInclusionInclusion = 1E10;
		private const double specialHeatCoeff = 1.0;

		public static void PlotModel()
		{
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
			model.GeometryModel.Enricher.Observers.Add(new PhaseEnrichmentPlotter(outputDirectory, model, elementSize, 3));

			// Initialize model state so that everything described above can be tracked
			model.Initialize();
		}

		//public static void RunHomogenization()
		//{
		//    // Create physical model, LSM and phases
		//    Console.WriteLine("Creating physical and geometric models");
		//    XModel<IXMultiphaseElement> model = CreateModel();
		//    model.FindConformingSubcells = true;
		//    PhaseGeometryModel geometryModel = CreatePhases(model);

		//    // Write the size of each phase
		//    model.ModelObservers.Add(new PhasesSizeWriter(outputDirectory, model, geometryModel));

		//    // Geometric interactions
		//    model.Initialize();

		//    // Run homogenization analysis
		//    IMatrix conductivity = Analysis.RunHomogenizationAnalysisThermal3D(model, minCoords, maxCoords);
		//    Console.WriteLine(
		//        $"conductivity = [ {conductivity[0, 0]} {conductivity[0, 1]} {conductivity[0, 2]};"
		//        + $" {conductivity[1, 0]} {conductivity[1, 1]} {conductivity[1, 2]};"
		//        + $" {conductivity[2, 0]} {conductivity[2, 1]} {conductivity[2, 2]} ]");
		//}

		private static XModel<IXMultiphaseElement> CreateModel()
		{
			// Materials
			var matrixMaterial = new ThermalMaterial(conductMatrix, specialHeatCoeff);
			var inclusionMaterial = new ThermalMaterial(conductInclusion, specialHeatCoeff);
			var materialField = new MatrixInclusionsThermalMaterialField(matrixMaterial, inclusionMaterial,
				conductBoundaryMatrixInclusion, conductBoundaryInclusionInclusion, defaultPhaseID);

			var model = Models.CreateHexa8Model(minCoords, maxCoords, numElements, true,
				bulkIntegrationOrder, boundaryIntegrationOrder, materialField, true);
			Models.ApplyBCsTemperatureDiffAlongX(model, +100, -100);

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

			// Geometry
			var ballsInternal = new List<Sphere>();
			//var ballsExternal = new List<Sphere>();
			double r = (maxCoords[0] - minCoords[0]) / (2 * numBalls[0]);
			for (int i = 0; i < numBalls[0]; ++i)
			{
				for (int j = 0; j < numBalls[1]; ++j)
				{
					for (int k = 0; k < numBalls[2]; ++k)
					{
						double[] center = 
						{ 
							minCoords[0] + (2 * i + 1) * r, 
							minCoords[1] + (2 * j + 1) * r, 
							minCoords[2] + (2 * k + 1) * r 
						};
						ballsInternal.Add(new Sphere(center, internalRadius));
						//ballsExternal.Add(new Sphere(center, externalRadius));
					}
				}
			}

			// Phases
			for (int p = 0; p < ballsInternal.Count; ++p)
			{
				//var externalPhase = new HollowLsmPhase(2 * p + 1, geometricModel, 0);
				//var externalCurve = new SimpleLsm3D(externalPhase.ID, model.XNodes, ballsExternal[p]);
				//geometricModel.Phases[externalPhase.ID] = externalPhase;

				//var externalBoundary = new ClosedLsmPhaseBoundary(externalPhase.ID, externalCurve, defaultPhase, externalPhase);
				//defaultPhase.ExternalBoundaries.Add(externalBoundary);
				//defaultPhase.Neighbors.Add(externalPhase);
				//externalPhase.ExternalBoundaries.Add(externalBoundary);
				//externalPhase.Neighbors.Add(defaultPhase);
				//geometricModel.PhaseBoundaries[externalBoundary.ID] = externalBoundary;

				//var internalLsm = new SimpleLsm3D(2 * p + 2, model.XNodes, ballsInternal[p]);
				//var internalPhase = new LsmPhase(2 * p + 2, geometricModel, -1);
				//geometricModel.Phases[internalPhase.ID] = internalPhase;

				//var internalBoundary = new ClosedLsmPhaseBoundary(internalPhase.ID, internalLsm, externalPhase, internalPhase);
				//externalPhase.InternalBoundaries.Add(internalBoundary);
				//externalPhase.InternalPhases.Add(internalPhase);
				//externalPhase.Neighbors.Add(internalPhase);
				//internalPhase.ExternalBoundaries.Add(internalBoundary);
				//internalPhase.Neighbors.Add(externalPhase);

				var internalLsm = new SimpleLsm3D(p+1, model.Nodes.Values.OrderBy(n => n.ID).ToList(), ballsInternal[p]);
				var internalPhase = new LsmPhase(p+1, geometricModel, 0);
				geometricModel.Phases[internalPhase.ID] = internalPhase;

				var internalBoundary = new ClosedPhaseBoundary(internalPhase.ID, internalLsm, defaultPhase, internalPhase);
				defaultPhase.Neighbors.Add(internalPhase);
				internalPhase.ExternalBoundaries.Add(internalBoundary);
				internalPhase.Neighbors.Add(defaultPhase);
			}
			return geometricModel;
		}

		
	}
}
