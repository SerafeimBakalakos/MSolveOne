using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class PolarCoordsAtPointsPlotter : ICrackObserver
	{
		private readonly string outputDirectory;
		private readonly UniformPointGenerator pointGenerator;
		private readonly IHybridFriesCrackDescription crack;
		private readonly TipCoordinateSystemImplicit tipCoordinateSystem;
		private readonly TipCoordinateSystemImplicitFries tipCoordinateSystemFries;
		private int iteration;

		public PolarCoordsAtPointsPlotter(UniformPointGenerator pointGenerator, IHybridFriesCrackDescription crack, 
			string outputDirectory)
		{
			this.crack = crack;
			this.pointGenerator = pointGenerator;
			this.outputDirectory = outputDirectory;
			this.tipCoordinateSystem = new TipCoordinateSystemImplicit(crack);
			this.tipCoordinateSystemFries = new TipCoordinateSystemImplicitFries(crack);

			iteration = 0;
		}

		public void Update()
		{

			List<double[]> pointsGlobal = pointGenerator.GeneratePointsGlobalCartesian();
			List<XPoint> pointsNatural = pointGenerator.GeneratePointsNatural(pointsGlobal);
			(List<double> r, List<double> theta) = CalcPolarCoords(tipCoordinateSystem, pointsNatural);

			string path = $"{outputDirectory}\\polar_coords_{crack.ID}_t{iteration}.vtk";
			using (var writer = new VtkPointWriter(path))
			{
				writer.WritePoints(pointsGlobal, true);
				writer.WriteScalarField("r", r);
				writer.WriteScalarField("theta", theta);
			}

			(List<double> rFries, List<double> thetaFries) = CalcPolarCoords(tipCoordinateSystemFries, pointsNatural);
			string pathFries = $"{outputDirectory}\\polar_coords_fries{crack.ID}_t{iteration}.vtk";
			using (var writer = new VtkPointWriter(pathFries))
			{
				writer.WritePoints(pointsGlobal, true);
				writer.WriteScalarField("r", rFries);
				writer.WriteScalarField("theta", thetaFries);
			}

			++iteration;
		}

		private static (List<double> r, List<double> theta) CalcPolarCoords(ITipCoordinateSystem tipCoordinateSystem, 
			List<XPoint> pointsNatural)
		{
			var allR = new List<double>(pointsNatural.Count);
			var allTheta = new List<double>(pointsNatural.Count);
			foreach (XPoint point in pointsNatural)
			{
				point.ShapeFunctions = point.Element.Interpolation.EvaluateFunctionsAt(
					point.Coordinates[CoordinateSystem.ElementNatural]);
				double[] polarCoords = tipCoordinateSystem.MapPointToLocalPolar(point);
				allR.Add(polarCoords[0]);
				allTheta.Add(polarCoords[1]);
			}
			return (allR, allTheta);
		}
	}
}
