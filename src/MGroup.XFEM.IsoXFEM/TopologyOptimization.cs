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
using System.Linq;
using MGroup.XFEM.IsoXFEM.SolidRatioComputations;
using MGroup.NumericalAnalyzers;
using MGroup.MSolve.Solution;
using MGroup.Solvers.Direct;
using MGroup.Solvers.AlgebraicModel;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Constitutive.Structural;

namespace MGroup.XFEM.IsoXFEM
{
    public  class TopologyOptimization
    {

        private const double volumeFraction = 0.5;
        private const double evolutionRate = 0.01;
        private const int iterations = 200;
        public /*private*/  Vector nodalStrainEnergyIt;
        private  Vector nodalStrainEnergyItPrevious;
		public /*private*/ double mlp;
        private  Vector vfEachIteration;
		public  Matrix results;
		private readonly StructuralPerfomance structuralPerfomance;
		private readonly ISolidRatio solidRatio;
		private /*readonly*/ ISolver solver;
		public XModel<IIsoXfemElement> xModel;
		public StaticAnalyzer parentAnalyzer;
		public IAlgebraicModel algebraicModel;

		public TopologyOptimization (XModel<IIsoXfemElement> xModel,ISolidRatio solidRatio,  StaticAnalyzer parentAnalyzer, ISolver solver, IAlgebraicModel algebraicModel)
		{
			this.xModel = xModel;
			this.parentAnalyzer = parentAnalyzer;
			this.solver = solver;
			this.algebraicModel = algebraicModel;
			this.structuralPerfomance = new StructuralPerfomance(xModel.Dimension,xModel.Nodes, xModel.Elements, xModel.Elements.First().Value.SizeOfElement, algebraicModel);
			this.solidRatio = solidRatio;
			parentAnalyzer.Initialize();
		}
        public void IsoXfem()
        {
            nodalStrainEnergyIt = Vector.CreateZero(xModel.Nodes.Count);
            nodalStrainEnergyItPrevious = Vector.CreateZero(xModel.Nodes.Count);
            Vector initialSizeOfElements = Vector.CreateWithValue(xModel.Elements.Count, xModel.Elements[0].SizeOfElement);
            var initialSizeElement = initialSizeOfElements[0];
			xModel.sizesOfElements = Vector.CreateWithValue(xModel.Elements.Count, initialSizeElement);
			vfEachIteration = Vector.CreateZero(iterations);
            var sizeOfWholeStructure = initialSizeOfElements.Sum();
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
					var provider = new ProblemStructural(xModel, algebraicModel, solver);
					var childAnalyzer = new LinearAnalyzer(xModel, algebraicModel, solver, provider);
					parentAnalyzer = new StaticAnalyzer(xModel, algebraicModel, solver, provider, childAnalyzer);
					parentAnalyzer.Initialize(false);
				}
				parentAnalyzer.Solve();
				var solution = solver.LinearSystem.Solution; 
				(var strainEnergy, var nodalStrainEnergyDensity) =structuralPerfomance.ComputeStrainEnergyAndNodalSEDensity(solution);
                nodalStrainEnergyIt = nodalStrainEnergyDensity;
                double totalStrainEnergy = strainEnergy.Sum();
                StabilizingEvolutionaryProcess(it);
                //Initialize Minimum Level Of Perfomance
                if (it == 0)
                {
                    double initialNodalStrainEnergyMaxValue = nodalStrainEnergyIt.Max();
                    mlp = 0.99 * initialNodalStrainEnergyMaxValue;
                }
                //Results For Txt Files
                vfEachIteration[it] = xModel.sizesOfElements.Sum() / sizeOfWholeStructure;
                results[it, 0] = it;
                results[it, 1] = totalStrainEnergy;
                results[it, 2] = vfEachIteration[it];
                //Target Volume Fraction for this iteration
                Vector casesForVolumeFractionsIt = Vector.CreateFromArray(new double[] { volumeFraction, vfi * (1 - evolutionRate) });
                vfi = casesForVolumeFractionsIt.Max();
                var relativeCriteria = UpdatingMLP(vfi, vfk, sizeOfWholeStructure);
				xModel.relativeCriteria = relativeCriteria;
				xModel.Update(null,null);
				//PlotPerformanceLevel(it, xModel.Nodes, xModel.Elements, relativeCriteria);
			}
			ResultsWriter.ResultsWriterToTxt(results);
		}
        private  void StabilizingEvolutionaryProcess(int iteration)
        {
            if (iteration > 0)
            {
                for (int i = 0; i < nodalStrainEnergyItPrevious.Length; i++)
                {
                    nodalStrainEnergyIt[i] = (nodalStrainEnergyIt[i] + nodalStrainEnergyItPrevious[i]) * (0.5);
                }
            }
        }
        public  Vector UpdatingMLP( double targetVolumeFraction, double volumeFractionIteration, double wholeSize)
        {
            Vector relativeCriteria = Vector.CreateZero(nodalStrainEnergyIt.Length);
            Vector mlpVector = Vector.CreateZero(nodalStrainEnergyIt.Length);
            while ((Math.Abs(targetVolumeFraction - volumeFractionIteration) / targetVolumeFraction) > 0.001)
            {
                for (int i = 0; i < nodalStrainEnergyIt.Length; i++)
                {
                    mlpVector[i] = mlp;
                }
                relativeCriteria = nodalStrainEnergyIt - mlpVector;
				solidRatio.RelativeCriteria = relativeCriteria;
				var sizeOfElements = solidRatio.CalculateSolidRatio();
				volumeFractionIteration = sizeOfElements.Sum() / wholeSize;
                mlp = mlp * volumeFractionIteration / targetVolumeFraction;
            }
            return relativeCriteria;
        }
   # region solidArea function
		//public static Vector SolidArea(XModel<IsoXfemElement2D> xModel, Vector initialAreasOfElements, Vector relativeCriteria)
  //      {
  //          Vector newArea = Vector.CreateZero(initialAreasOfElements.Length);
  //          double areaRatio = 1.00;
  //          for (int i = 0; i < xModel.Elements.Count; i++)
  //          {
  //              int[] connectionOfElement = new int[] { xModel.Elements[i].nodesOfElement[0].ID, xModel.Elements[i].nodesOfElement[1].ID, xModel.Elements[i].nodesOfElement[2].ID, xModel.Elements[i].nodesOfElement[3].ID };
  //              if (relativeCriteria.GetSubvector(connectionOfElement).Min() > 0)//solid element
  //              {
  //                  areaRatio = 1;
  //              }
  //              else if (relativeCriteria.GetSubvector(connectionOfElement).Max() < 0)// void element
  //              {
  //                  areaRatio = 0;
  //              }
  //              else //boundary element
  //              {
  //                  Matrix s = Matrix.CreateZero(21, 21);
  //                  Matrix t = Matrix.CreateZero(21, 21);
  //                  for (int j = 0; j < 21; j++)
  //                  {
  //                      double v = -1;
  //                      for (int k = 0; k < 21; k++)
  //                      {
  //                          t[k, j] = v;
  //                          s[j, k] = v;
  //                          v = v + 0.1;
  //                      }
  //                  }
  //                  int m = 0;
  //                  Vector tmpPhi = Vector.CreateZero(441);
  //                  for (int j = 0; j < 21; j++)
  //                  {
  //                      for (int k = 0; k < 21; k++)
  //                      {
  //                          tmpPhi[m] = (1 - s[j, k]) * (1 - t[j, k]) * 0.25 * relativeCriteria[connectionOfElement[0]] + (1 + s[j, k]) * (1 - t[j, k]) * 0.25 * relativeCriteria[connectionOfElement[1]] + (1 + s[j, k]) * (1 + t[j, k]) * 0.25 * relativeCriteria[connectionOfElement[2]] + (1 - s[j, k]) * (1 + t[j, k]) * 0.25 * relativeCriteria[connectionOfElement[3]];
  //                          m++;

  //                      }
  //                  }
  //                  double numofNoNegative = 0;
  //                  for (int j = 0; j < tmpPhi.Length; j++)
  //                  {
  //                      if (tmpPhi[j] >= 0.00)
  //                      {
  //                          numofNoNegative++;
  //                      }
  //                  }
  //                  areaRatio = numofNoNegative / tmpPhi.Length;
  //              }
  //              newArea[i] = areaRatio * initialAreasOfElements[i];
  //          }
  //          return newArea;
  //      }
		#endregion
		public static void PlotPerformanceLevel(int iteration, Dictionary<int, XNode>  nodes, Dictionary<int, IIsoXfemElement>  elements, Vector nodalValues)
        {                    
            string path = $"{ Paths.OutputDirectory}\\OOS_12_BottomEnd_40x20_SkylineLDL_InitialStiffness_ComputeOnlyOneTime_CorrectMatlabErrors{iteration}.vtk";
            var writer = new VtkFileWriter(path);
            writer.WriteMesh(nodes, elements);
            writer.WriteScalarField("performance_level", nodalValues.RawData);

            writer.Dispose();
        }
    }
}
