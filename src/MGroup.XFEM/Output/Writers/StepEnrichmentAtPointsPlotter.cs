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
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Interpolation;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class StepEnrichmentAtPointsPlotter : ICrackObserver
	{
		private readonly string outputDirectory;
		private readonly UniformPointGenerator pointGenerator;
		private readonly IHybridFriesCrackDescription crack;
		private readonly CrackStepEnrichment_v2 stepEnrichment;
		private int iteration;

		public StepEnrichmentAtPointsPlotter(UniformPointGenerator pointGenerator,
			IHybridFriesCrackDescription crack, string outputDirectory)
		{
			this.crack = crack;
			this.pointGenerator = pointGenerator;
			this.outputDirectory = outputDirectory;

			this.stepEnrichment = new CrackStepEnrichment_v2(crack);

			iteration = 0;
		}

		public void Update()
		{
			List<double[]> pointsGlobal = pointGenerator.GeneratePointsGlobalCartesian();
			List<XPoint> pointsNatural = pointGenerator.GeneratePointsNatural(pointsGlobal);

			List<double> functions = CalcStepFunctions(pointsNatural, stepEnrichment);
			string pathFunctions = $"{outputDirectory}\\heaviside_enrichment_functions_{crack.ID}_t{iteration}.vtk";
			using (var writer = new VtkPointWriter(pathFunctions))
			{
				writer.WritePoints(pointsGlobal, true);
				writer.WriteScalarField("H", functions);
			}
			
			++iteration;
		}

		private static List<double> CalcStepFunctions(List<XPoint> pointsNatural, CrackStepEnrichment_v2 stepEnrichment)
		{
			var h = new List<double>(pointsNatural.Count);
			foreach (XPoint point in pointsNatural)
			{
				point.ShapeFunctions = point.Element.Interpolation.EvaluateFunctionsAt(
					point.Coordinates[CoordinateSystem.ElementNatural]);
				h.Add(stepEnrichment.EvaluateAt(point));
			}
			return h;
		}
	}
}
