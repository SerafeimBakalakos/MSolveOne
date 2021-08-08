using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.XFEM.Tests.Utilities;

namespace MGroup.XFEM.Tests.MultiphaseThermal.EpoxyAg
{
    public class ExampleStochastic2D
    {
        private const string outputDirectory = @"C:\Users\Serafeim\Desktop\HEAT\2020\EpoxyAG\Stochastic2D\";
        private const string pathResults = outputDirectory + "results.txt";

        private static readonly double[] minCoords = { -1.0, -1.0 };
        private static readonly double[] maxCoords = { +1.0, +1.0 };
        private const double thickness = 1.0;
        private static readonly int[] numElements = { 45, 45 };
        private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;

        private const int numBalls = 8;
        private const double epoxyPhaseRadius = 0.2, silverPhaseThickness = 0.1;

        private const double conductEpoxy = 1E0, conductSilver = 1E2;
        private const double conductBoundaryEpoxySilver = 1E1;
        private const double specialHeatCoeff = 1.0;

        private readonly int rngSeed;

        public ExampleStochastic2D(int rngSeed)
        {
            this.rngSeed = rngSeed;
        }

        public static void RunAll()
        {
            int numRealizations = 10;
            int initialSeed = 33;
            var rng = new Random(initialSeed);
            for (int i = 0; i < numRealizations; i++)
            {
                Console.WriteLine();
                Console.WriteLine("Realization " + i);
                int realizationSeed = rng.Next();
                var realization = new ExampleStochastic2D(realizationSeed);
                realization.Run();
            }
        }

        public void Run()
        {
            //try
            //{
            //    // Create physical model, LSM and phases
            //    (XModel<IXMultiphaseElement> model, ThermalBiMaterialField materialField) = CreateModel();
            //    model.FindConformingSubcells = true;
            //    GeometryPreprocessor2D geometryPreprocessor = CreatePhases(model, materialField);

            //    // Initialize model
            //    Console.WriteLine("Initialize model");
            //    model.Initialize();

            //    // Run homogenization analysis
            //    IMatrix conductivity = Analysis.RunHomogenizationAnalysisThermal2D(model, minCoords, maxCoords, thickness);

            //    // Print results
            //    using (var writer = new StreamWriter(pathResults, true))
            //    {
            //        writer.WriteLine();
            //        writer.WriteLine("#################################################################");
            //        writer.WriteLine("Date = " + DateTime.Now);
            //        writer.WriteLine("Realization with seed = " + rngSeed);
            //        string volumesString = geometryPreprocessor.PrintVolumes();
            //        writer.WriteLine(volumesString);
            //        writer.WriteLine(
            //            $"conductivity = [ {conductivity[0, 0]} {conductivity[0, 1]}; {conductivity[1, 0]} {conductivity[1, 1]} ]");
            //    }
            //}
            //catch (Exception)
            //{
            //    using (var writer = new StreamWriter(pathResults, true))
            //    {
            //        writer.WriteLine();
            //        writer.WriteLine("#################################################################");
            //        writer.WriteLine("Realization with seed = " + rngSeed);
            //        writer.WriteLine("Analysis failed!");
            //    }
            //}
        }

        private static (XModel<IXMultiphaseElement>, ThermalBiMaterialField) CreateModel()
        {
            // Materials
            var epoxyMaterial = new ThermalMaterial(conductEpoxy, specialHeatCoeff);
            var silverMaterial = new ThermalMaterial(conductSilver, specialHeatCoeff);
            var materialField = new ThermalBiMaterialField(epoxyMaterial, silverMaterial, conductBoundaryEpoxySilver);

            return (Models.CreateQuad4Model(minCoords, maxCoords, thickness, numElements, true,
                bulkIntegrationOrder, boundaryIntegrationOrder, materialField, true), materialField);
        }

        private GeometryPreprocessor2D CreatePhases(XModel<IXMultiphaseElement> model, ThermalBiMaterialField materialField)
        {
            var preprocessor = new GeometryPreprocessor2D(model);
            preprocessor.MinCoordinates = minCoords;
            preprocessor.MaxCoordinates = maxCoords;
            preprocessor.NumBalls = numBalls;
            preprocessor.RngSeed = rngSeed;
            preprocessor.RadiusEpoxyPhase = epoxyPhaseRadius;
            preprocessor.ThicknessSilverPhase = silverPhaseThickness;

            preprocessor.GeneratePhases(model);
            materialField.PhasesWithMaterial0.Add(preprocessor.MatrixPhaseID);
            foreach (int p in preprocessor.EpoxyPhaseIDs) materialField.PhasesWithMaterial0.Add(p);
            foreach (int p in preprocessor.SilverPhaseIDs) materialField.PhasesWithMaterial1.Add(p);

            return preprocessor;
        }
    }
}
