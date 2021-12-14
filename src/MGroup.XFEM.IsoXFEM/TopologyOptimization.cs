using MGroup.XFEM.IsoXFEM.Output;
using MGroup.XFEM.IsoXFEM.ElementStructuralStiffnessComputations;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Output;
using MGroup.LinearAlgebra.Output.Formatting;
using MGroup.LinearAlgebra.Reduction;
using MGroup.LinearAlgebra.Vectors;
using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.IsoXFEM
{
    public static class TopologyOptimization
    {

        private const double volumeFraction = 0.5;
        private const double evolutionRate = 0.01;
        private const int iterations = 200;
        public /*private*/ static Vector nodalStrainEnergyIt;
        private static Vector nodalStrainEnergyItPrevious;
		public /*private*/static double mlp;
        private static Vector vfEachIteration;
		public static Matrix results;
        public static void IsoXfem(XModel<IsoXfemElement2D> xModel, FEMAnalysis femAnalysis)
        {
            nodalStrainEnergyIt = Vector.CreateZero(xModel.Nodes.Count);
            nodalStrainEnergyItPrevious = Vector.CreateZero(xModel.Nodes.Count);
            Vector initialAreaOfElements = Vector.CreateWithValue(xModel.Elements.Count, xModel.Elements[0].areaOfElement);
            var initialAreaElement = initialAreaOfElements[0];
            Vector areaOfElements = Vector.CreateWithValue(xModel.Elements.Count, initialAreaElement);
            vfEachIteration = Vector.CreateZero(iterations);
            var areaOfWholeStructure = initialAreaOfElements.Sum();
            var vfi = 1.00;
            var vfk = 0.00;
            results = Matrix.CreateZero(iterations, 3);            
            for (int it = 0; it < iterations; it++)
            {
                if (it > 0)
                {
                    for (int i = 0; i < nodalStrainEnergyItPrevious.Length; i++)
                    {
                        nodalStrainEnergyItPrevious[i] = nodalStrainEnergyIt[i];
                    }
                }
                femAnalysis.Solve();			
				var structuralPerfomance = new StructuralPerfomance(xModel.Nodes, xModel.Elements, initialAreaElement, femAnalysis.displacements);
				xModel.levelSetStructuralPerfomance = structuralPerfomance;
				xModel.Update();
                //structuralPerfomance.ComputeStrainEnergyandStrainEnergyDensity();
                //structuralPerfomance.ComputeNodalStrainEnergyDensity();
                nodalStrainEnergyIt = structuralPerfomance.nodalStrainEnergyDensity;
                double totalStrainEnergy = structuralPerfomance.strainEnergy.Sum();
                StabilizingEvolutionaryProcess(it);
                //Initialize Minimum Level Of Perfomance
                if (it == 0)
                {
                    double initialNodalStrainEnergyMaxValue = nodalStrainEnergyIt.Max();
                    mlp = 0.99 * initialNodalStrainEnergyMaxValue;
                }
                //Results For Txt Files
                vfEachIteration[it] = areaOfElements.Sum() / areaOfWholeStructure;
                results[it, 0] = it;
                results[it, 1] = totalStrainEnergy;
                results[it, 2] = vfEachIteration[it];
                //Target Volume Fraction for this iteration
                Vector casesForVolumeFractionsIt = Vector.CreateFromArray(new double[] { volumeFraction, vfi * (1 - evolutionRate) });
                vfi = casesForVolumeFractionsIt.Max();
                var relativeCriteria = UpdatingMLP(xModel, vfi, vfk, initialAreaOfElements, areaOfWholeStructure);				
				//PlotPerformanceLevel(it, model.nodes, model.elements, relativeCriteria);
				for (int el = 0; el < xModel.Elements.Count; el++)
                {
                    int[] connectionOfElement = new int[] { xModel.Elements[el].nodesOfElement[0].ID, xModel.Elements[el].nodesOfElement[1].ID, xModel.Elements[el].nodesOfElement[2].ID, xModel.Elements[el].nodesOfElement[3].ID };
                    Vector elementRelativeCriteria = relativeCriteria.GetSubvector(connectionOfElement);
                    Matrix elementCoordinates = Matrix.CreateFromArray(new double[,] {{ xModel.Elements[el].nodesOfElement[0].X, xModel.Elements[el].nodesOfElement[0].Y},
                                                                                       {xModel.Elements[el].nodesOfElement[1].X, xModel.Elements[el].nodesOfElement[1].Y },
                                                                                       {xModel.Elements[el].nodesOfElement[2].X, xModel.Elements[el].nodesOfElement[2].Y },
                                                                                       {xModel.Elements[el].nodesOfElement[3].X, xModel.Elements[el].nodesOfElement[3].Y }});
					xModel.Elements[el].coordinatesOfElement = elementCoordinates;
					xModel.Elements[el].elementLevelSet = elementRelativeCriteria;
					xModel.Elements[el].StiffnessMatrix(xModel.Elements[el]);
					areaOfElements[el] = xModel.Elements[el].areaOfElement;
                }
            }
            ResultsWriter.ResultsWriterToTxt(results);
        }
        private static void StabilizingEvolutionaryProcess(int iteration)
        {
            if (iteration > 0)
            {
                for (int i = 0; i < nodalStrainEnergyItPrevious.Length; i++)
                {
                    nodalStrainEnergyIt[i] = (nodalStrainEnergyIt[i] + nodalStrainEnergyItPrevious[i]) * (0.5);
                }
            }
        }
        public static Vector UpdatingMLP(XModel<IsoXfemElement2D> xModel, double targetVolumeFraction, double volumeFractionIteration, Vector initialAreas, double wholeArea)
        {
            int z = 0;
            Vector relativeCriteria = Vector.CreateZero(nodalStrainEnergyIt.Length);
            Vector mlpVector = Vector.CreateZero(nodalStrainEnergyIt.Length);
            while ((Math.Abs(targetVolumeFraction - volumeFractionIteration) / targetVolumeFraction) > 0.001)
            {
                for (int i = 0; i < nodalStrainEnergyIt.Length; i++)
                {
                    mlpVector[i] = mlp;
                }
                relativeCriteria = nodalStrainEnergyIt - mlpVector;
                var areasofelement = SolidArea(xModel, initialAreas, relativeCriteria);
                volumeFractionIteration = areasofelement.Sum() / wholeArea;
                mlp = mlp * volumeFractionIteration / targetVolumeFraction;
                z++;
            }
            return relativeCriteria;
        }
        public static Vector SolidArea(XModel<IsoXfemElement2D> xModel, Vector initialAreasOfElements, Vector relativeCriteria)
        {
            Vector newArea = Vector.CreateZero(initialAreasOfElements.Length);
            double areaRatio = 1.00;
            for (int i = 0; i < xModel.Elements.Count; i++)
            {
                int[] connectionOfElement = new int[] { xModel.Elements[i].nodesOfElement[0].ID, xModel.Elements[i].nodesOfElement[1].ID, xModel.Elements[i].nodesOfElement[2].ID, xModel.Elements[i].nodesOfElement[3].ID };
                if (relativeCriteria.GetSubvector(connectionOfElement).Min() > 0)
                {
                    areaRatio = 1;
                }
                else if (relativeCriteria.GetSubvector(connectionOfElement).Max() < 0)
                {
                    areaRatio = 0;
                }
                else
                {
                    Matrix s = Matrix.CreateZero(21, 21);
                    Matrix t = Matrix.CreateZero(21, 21);
                    for (int j = 0; j < 21; j++)
                    {
                        double v = -1;
                        for (int k = 0; k < 21; k++)
                        {
                            t[k, j] = v;
                            s[j, k] = v;
                            v = v + 0.1;
                        }
                    }
                    int m = 0;
                    Vector tmpPhi = Vector.CreateZero(441);
                    for (int j = 0; j < 21; j++)
                    {
                        for (int k = 0; k < 21; k++)
                        {
                            tmpPhi[m] = (1 - s[j, k]) * (1 - t[j, k]) * 0.25 * relativeCriteria[connectionOfElement[0]] + (1 + s[j, k]) * (1 - t[j, k]) * 0.25 * relativeCriteria[connectionOfElement[1]] + (1 + s[j, k]) * (1 + t[j, k]) * 0.25 * relativeCriteria[connectionOfElement[2]] + (1 - s[j, k]) * (1 + t[j, k]) * 0.25 * relativeCriteria[connectionOfElement[3]];
                            m++;

                        }
                    }
                    double numofNoNegative = 0;
                    for (int j = 0; j < tmpPhi.Length; j++)
                    {
                        if (tmpPhi[j] >= 0.00)
                        {
                            numofNoNegative++;
                        }
                    }
                    areaRatio = numofNoNegative / tmpPhi.Length;
                }
                newArea[i] = areaRatio * initialAreasOfElements[i];
            }
            return newArea;
        }
        public static void PlotPerformanceLevel(int iteration, Dictionary<int, XNode>  nodes , List<IsoXfemElement2D> elements, Vector nodalValues)
        {                    
            string path = $"{ Paths.OutputDirectory}\\OOS2BottomEnd_40x20_SkylineLDL_InitialStiffness_ComputeOnlyOneTime_CorrectMatlabErrors{iteration}.vtk";
            var writer = new VtkFileWriter(path);
            writer.WriteMesh(nodes, elements);
            writer.WriteScalarField("performance_level", nodalValues.RawData);

            writer.Dispose();
        }
    }
}
