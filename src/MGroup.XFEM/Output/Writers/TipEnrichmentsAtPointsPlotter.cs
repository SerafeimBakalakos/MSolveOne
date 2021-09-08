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
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class TipEnrichmentsAtPointsPlotter : ICrackObserver
	{
		private readonly string outputDirectory;
		private readonly UniformPointGenerator pointGenerator;
		private readonly IHybridFriesCrackDescription crack;
		private readonly ICrackTipEnrichment[] tipEnrichments;
		private readonly ICrackTipEnrichment[] tipEnrichmentsFries;
		private int iteration;

		public TipEnrichmentsAtPointsPlotter(UniformPointGenerator pointGenerator, IHybridFriesCrackDescription crack, 
			string outputDirectory)
		{
			this.crack = crack;
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
			string pathFunctions = $"{outputDirectory}\\tip_enrichment_functions_{crack.ID}_t{iteration}.vtk";
			string pathFunctionsFries = $"{outputDirectory}\\tip_enrichment_functions_fries_{crack.ID}_t{iteration}.vtk";
			string pathGradients = $"{outputDirectory}\\tip_enrichment_gradients_{crack.ID}_t{iteration}.vtk";

			List<double[]> pointsGlobal = pointGenerator.GeneratePointsGlobalCartesian();
			List<XPoint> pointsNatural = pointGenerator.GeneratePointsNatural(pointsGlobal);
			var enrichments = CalcTipFunctions(pointsNatural, tipEnrichments);
			var enrichmentsFries = CalcTipFunctions(pointsNatural, tipEnrichmentsFries);

			using (var writer = new VtkPointWriter(pathFunctions))
			{
				writer.WritePoints(pointsGlobal, true);
				writer.WriteScalarField("f1", enrichments.f0);
				writer.WriteScalarField("f2", enrichments.f1);
				writer.WriteScalarField("f3", enrichments.f2);
				writer.WriteScalarField("f4", enrichments.f3);
			}

			using (var writer = new VtkPointWriter(pathFunctionsFries))
			{
				writer.WritePoints(pointsGlobal, true);
				writer.WriteScalarField("f1", enrichmentsFries.f0);
				writer.WriteScalarField("f2", enrichmentsFries.f1);
				writer.WriteScalarField("f3", enrichmentsFries.f2);
				writer.WriteScalarField("f4", enrichmentsFries.f3);
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
	}
}
