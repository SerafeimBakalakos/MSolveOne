using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class CoordinatesAtGaussPointsPlotter : IModelObserver
	{
		private readonly IXModel model;
		private readonly IHybridFriesCrackDescription crack;
		private readonly ITipCoordinateSystem tipCoordinateSystem;
		private readonly string outputDirectory;
		private int iteration;

		public CoordinatesAtGaussPointsPlotter(IXModel model, IHybridFriesCrackDescription crack, 
			ITipCoordinateSystem tipCoordinateSystem, string outputDirectory)
		{
			this.model = model;
			this.crack = crack;
			this.tipCoordinateSystem = tipCoordinateSystem;
			this.outputDirectory = outputDirectory;
		}

		public void Update()
		{
			PlotLevelSets();
			PlotPolarCoords();

			++iteration;
		}

		private void PlotLevelSets()
		{
			string path = Path.Combine(outputDirectory, $"level_sets_at_gauss_points_t{iteration}.vtk");

			var globalCoords = new List<double[]>();
			var phi = new List<double>();
			var psi = new List<double>();
			var regions = new List<double>();

			foreach (IXFiniteElement element in model.EnumerateElements())
			{
				foreach (GaussPoint gp in element.BulkIntegrationPoints)
				{
					var point = new XPoint(model.Dimension);

					point.Element = element;
					point.Coordinates[CoordinateSystem.ElementNatural] = gp.Coordinates;
					point.ShapeFunctions = element.Interpolation.EvaluateFunctionsAt(gp.Coordinates);
					double[] coords = point.MapCoordinates(point.ShapeFunctions, element.Nodes);
					globalCoords.Add(coords);

					#region debug
					if (coords[0] > 0.917 && coords[0] < 0.95 && coords[1] > 1.82 && coords[1] < 1.833)
					{
						Console.WriteLine();
					}
					#endregion

					double[] regularLevelSets = crack.InterpolateLevelSets(point);
					phi.Add(regularLevelSets[0]);
					psi.Add(regularLevelSets[1]);

					double[] tripleLevelSets = crack.InterpolateTripleLevelSets(point);
					CrackedDomainRegion region = AuxiliaryCoordinateSystems.DetermineRegion(tripleLevelSets);
					regions.Add((int)region);
				}
			}

			using (var writer = new VtkPointWriter(path))
			{
				writer.WritePoints(globalCoords, true);
				writer.WriteScalarField("phi", phi);
				writer.WriteScalarField("psi", psi);
				writer.WriteScalarField("region", regions);
			}
		}

		private void PlotPolarCoords()
		{
			string path = Path.Combine(outputDirectory, $"polar_coords_at_gauss_points_t{iteration}.vtk");

			var globalCoords = new List<double[]>();
			var r = new List<double>();
			var theta = new List<double>();

			foreach (IXFiniteElement element in model.EnumerateElements())
			{
				foreach (GaussPoint gp in element.BulkIntegrationPoints)
				{
					var point = new XPoint(model.Dimension);

					point.Element = element;
					point.Coordinates[CoordinateSystem.ElementNatural] = gp.Coordinates;
					point.ShapeFunctions = element.Interpolation.EvaluateFunctionsAt(gp.Coordinates);
					double[] coords = point.MapCoordinates(point.ShapeFunctions, element.Nodes);
					globalCoords.Add(coords);

					double[] polarCoords = tipCoordinateSystem.MapPointToLocalPolar(point);
					r.Add(polarCoords[0]);
					theta.Add(polarCoords[1]);
				}
			}

			using (var writer = new VtkPointWriter(path))
			{
				writer.WritePoints(globalCoords, true);
				writer.WriteScalarField("r", r);
				writer.WriteScalarField("theta", theta);
			}
		}
	}
}
