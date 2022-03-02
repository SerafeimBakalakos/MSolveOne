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
using MGroup.XFEM.IsoXFEM.IsoXfemElements;
using MGroup.MSolve.Discretization.Mesh;

namespace MGroup.XFEM.IsoXFEM
{
    public  class TopologyOptimization
    {

        private const double volumeFraction = 0.5;
        private const double evolutionRate = 0.01;
        private const int iterations = 200;
        public /*private*/  Vector nodalStrainEnergyIt;
        private  Vector nodalStrainEnergyItPrevious;
		public  double mlp;
        private Vector vfEachIteration;
		public  Matrix results;
		private readonly StructuralPerfomance structuralPerfomance;
		private readonly ISolidRatio solidRatio;
		private  StaticAnalyzer parentAnalyzer;
		private ISolver solver;
		public XModel<IIsoXfemElement> xModel;
		public IAlgebraicModel algebraicModel;

		public TopologyOptimization (XModel<IIsoXfemElement> xModel,ISolidRatio solidRatio,  ISolver solver, IAlgebraicModel algebraicModel)
		{
			this.xModel = xModel;
			this.solver = solver;
			this.algebraicModel = algebraicModel;
			this.structuralPerfomance = new StructuralPerfomance(xModel.Dimension,xModel.Nodes, xModel.Elements, xModel.Elements.First().Value.SizeOfElement, algebraicModel);
			this.solidRatio = solidRatio;
			var provider = new ProblemStructural(xModel, algebraicModel, solver);
			var childAnalyzer = new LinearAnalyzer(xModel, algebraicModel, solver, provider);
			parentAnalyzer = new StaticAnalyzer(xModel, algebraicModel, solver, provider, childAnalyzer);
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
				Console.WriteLine("iter=" + it);
				//ResultsWriter.VolumeForEachElementWriter(it, xModel.Dimension, xModel.Elements);
				//ResultsWriter.GaussPointsWriter(it,xModel.Dimension, xModel.Elements);
				//ResultsWriter.InteractionPoints3DWriter(it, xModel.Dimension, xModel.Elements);
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
                    mlp = (1-evolutionRate)* initialNodalStrainEnergyMaxValue;
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
				//ResultsWriter.NodalLevelSetsWriter(it, xModel.Dimension, xModel.Nodes, relativeCriteria);
				xModel.relativeCriteria = relativeCriteria;
				xModel.Update(null,null);
				PlotPerformanceLevel(it, xModel.Nodes, xModel.Elements, relativeCriteria);
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
		public  void PlotPerformanceLevel(int iteration, Dictionary<int, XNode>  nodes, Dictionary<int, IIsoXfemElement>  elements, Vector nodalValues)
        {                    
            string path = $"{ Paths.OutputDirectory}\\2DTriangulator_100x50_iter{iteration}.vtk";
			CellType cellType = xModel.Elements.First().Value.CellType;
            var writer = new VtkFileWriter(path, xModel.Dimension, cellType);
            writer.WriteMesh(nodes, elements);
            writer.WriteScalarField("performance_level", nodalValues.RawData);

            writer.Dispose();
        }
    }
}
