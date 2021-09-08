using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Interpolation;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class TipEnrichmentsAtPointsPlotter : ICrackObserver
	{
		private readonly string outputDirectory;
		private readonly int dimension;
		private readonly UniformPointGenerator pointGenerator;
		private readonly IHybridFriesCrackDescription crack;
		private readonly ICrackTipEnrichment[] tipEnrichments;
		private readonly ICrackTipEnrichment[] tipEnrichmentsFries;
		private int iteration;

		public TipEnrichmentsAtPointsPlotter(int dimension, UniformPointGenerator pointGenerator,
			IHybridFriesCrackDescription crack, string outputDirectory)
		{
			this.crack = crack;
			this.dimension = dimension;
			this.pointGenerator = pointGenerator;
			this.outputDirectory = outputDirectory;

			var tipCoordSystem = new TipCoordinateSystemImplicit(crack);
			this.tipEnrichments = new ICrackTipEnrichment[]
			{
				new IsotropicBrittleTipEnrichments_v2.Func0(tipCoordSystem),
				new IsotropicBrittleTipEnrichments_v2.Func1(tipCoordSystem),
				new IsotropicBrittleTipEnrichments_v2.Func2(tipCoordSystem),
				new IsotropicBrittleTipEnrichments_v2.Func3(tipCoordSystem)
			};

			var tipCoordSystemFries = new TipCoordinateSystemImplicitFries(crack);
			this.tipEnrichmentsFries = new ICrackTipEnrichment[]
			{
				new IsotropicBrittleTipEnrichments_v2.Func0(tipCoordSystemFries),
				new IsotropicBrittleTipEnrichments_v2.Func1(tipCoordSystemFries),
				new IsotropicBrittleTipEnrichments_v2.Func2(tipCoordSystemFries),
				new IsotropicBrittleTipEnrichments_v2.Func3(tipCoordSystemFries)
			};

			iteration = 0;
		}

		public void Update()
		{
			//HERE: Also plot the tip gradients and Heaviside functions
			List<double[]> pointsGlobal = pointGenerator.GeneratePointsGlobalCartesian();
			List<XPoint> pointsNatural = pointGenerator.GeneratePointsNatural(pointsGlobal);

			var functions = CalcTipFunctions(pointsNatural, tipEnrichments);
			string pathFunctions = $"{outputDirectory}\\tip_enrichment_functions_{crack.ID}_t{iteration}.vtk";
			using (var writer = new VtkPointWriter(pathFunctions))
			{
				writer.WritePoints(pointsGlobal, true);
				writer.WriteScalarField("f1", functions.f0);
				writer.WriteScalarField("f2", functions.f1);
				writer.WriteScalarField("f3", functions.f2);
				writer.WriteScalarField("f4", functions.f3);
			}
			
			var functionsFries = CalcTipFunctions(pointsNatural, tipEnrichmentsFries);
			string pathFunctionsFries = $"{outputDirectory}\\tip_enrichment_functions_fries_{crack.ID}_t{iteration}.vtk";
			using (var writer = new VtkPointWriter(pathFunctionsFries))
			{
				writer.WritePoints(pointsGlobal, true);
				writer.WriteScalarField("f1", functionsFries.f0);
				writer.WriteScalarField("f2", functionsFries.f1);
				writer.WriteScalarField("f3", functionsFries.f2);
				writer.WriteScalarField("f4", functionsFries.f3);
			}

			var gradients = CalcTipGradients(dimension, pointsNatural, tipEnrichments);
			string pathGradients = $"{outputDirectory}\\tip_enrichment_gradients_{crack.ID}_t{iteration}.vtk";
			using (var writer = new VtkPointWriter(pathGradients))
			{
				writer.WritePoints(pointsGlobal, true);
				for (int d = 0; d < dimension; ++d)
				{
					writer.WriteScalarField($"df1/dx{d+1}", gradients.gradF0[d]);
					writer.WriteScalarField($"df2/dx{d+1}", gradients.gradF1[d]);
					writer.WriteScalarField($"df3/dx{d+1}", gradients.gradF2[d]);
					writer.WriteScalarField($"df4/dx{d+1}", gradients.gradF3[d]);
				}
			}

			var gradientsFries = CalcTipGradients(dimension, pointsNatural, tipEnrichmentsFries);
			string pathGradientsFries = $"{outputDirectory}\\tip_enrichment_gradients_fries_{crack.ID}_t{iteration}.vtk";
			using (var writer = new VtkPointWriter(pathGradientsFries))
			{
				writer.WritePoints(pointsGlobal, true);
				for (int d = 0; d < dimension; ++d)
				{
					writer.WriteScalarField($"df1/dx{d + 1}", gradientsFries.gradF0[d]);
					writer.WriteScalarField($"df2/dx{d + 1}", gradientsFries.gradF1[d]);
					writer.WriteScalarField($"df3/dx{d + 1}", gradientsFries.gradF2[d]);
					writer.WriteScalarField($"df4/dx{d + 1}", gradientsFries.gradF3[d]);
				}
			}

			++iteration;
		}

		private static (List<double> f0, List<double> f1, List<double> f2, List<double> f3) CalcTipFunctions(
			List<XPoint> pointsNatural, ICrackTipEnrichment[] tipEnrichments)
		{
			var f0 = new List<double>(pointsNatural.Count);
			var f1 = new List<double>(pointsNatural.Count);
			var f2 = new List<double>(pointsNatural.Count);
			var f3 = new List<double>(pointsNatural.Count);
			foreach (XPoint point in pointsNatural)
			{
				point.ShapeFunctions = point.Element.Interpolation.EvaluateFunctionsAt(
					point.Coordinates[CoordinateSystem.ElementNatural]);
				f0.Add(tipEnrichments[0].EvaluateAt(point));
				f1.Add(tipEnrichments[1].EvaluateAt(point));
				f2.Add(tipEnrichments[2].EvaluateAt(point));
				f3.Add(tipEnrichments[3].EvaluateAt(point));
			}
			return (f0, f1, f2, f3);
		}

		private static (List<double>[] gradF0, List<double>[] gradF1, List<double>[] gradF2, List<double>[] gradF3) 
			CalcTipGradients(int dim, List<XPoint> pointsNatural, ICrackTipEnrichment[] tipEnrichments)
		{
			var gradF0 = new List<double>[dim];
			var gradF1 = new List<double>[dim];
			var gradF2 = new List<double>[dim];
			var gradF3 = new List<double>[dim];
			for (int d = 0; d < dim; ++d)
			{
				gradF0[d] = new List<double>(pointsNatural.Count);
				gradF1[d] = new List<double>(pointsNatural.Count);
				gradF2[d] = new List<double>(pointsNatural.Count);
				gradF3[d] = new List<double>(pointsNatural.Count);
			}

			foreach (XPoint point in pointsNatural)
			{

				EvalInterpolation interpolation = point.Element.Interpolation.EvaluateAllAt(
					point.Element.Nodes, point.Coordinates[CoordinateSystem.ElementNatural]);
				point.ShapeFunctionDerivatives = interpolation.ShapeGradientsCartesian;
				point.JacobianNaturalGlobal = interpolation.Jacobian;

				double[] df0 = tipEnrichments[0].EvaluateAllAt(point).CartesianDerivatives;
				double[] df1 = tipEnrichments[1].EvaluateAllAt(point).CartesianDerivatives;
				double[] df2 = tipEnrichments[2].EvaluateAllAt(point).CartesianDerivatives;
				double[] df3 = tipEnrichments[3].EvaluateAllAt(point).CartesianDerivatives;
				for (int d = 0; d < dim; ++d)
				{
					gradF0[d].Add(df0[d]);
					gradF1[d].Add(df1[d]);
					gradF2[d].Add(df2[d]);
					gradF3[d].Add(df3[d]);
				}
			}
			return (gradF0, gradF1, gradF2, gradF3);
		}
	}
}
