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
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class PolarCoordsAtPointsPlotter : ICrackObserver
	{
		private readonly string outputDirectory;
		private readonly IHybridFriesCrackDescription crack;
		private readonly XModel<IXCrackElement> model;
		private readonly ICartesianMesh mesh;
		private readonly int numPointsPerAxis;
		private readonly TipCoordinateSystemImplicit tipCoordinateSystem;
		private int iteration;

		public PolarCoordsAtPointsPlotter(string outputDirectory, IHybridFriesCrackDescription crack,
			XModel<IXCrackElement> model, ICartesianMesh mesh, int numPointsPerAxis)
		{
			this.outputDirectory = outputDirectory;
			this.crack = crack;
			this.model = model;
			this.mesh = mesh;
			this.numPointsPerAxis = numPointsPerAxis;
			this.tipCoordinateSystem = new TipCoordinateSystemImplicit(crack);

			iteration = 0;
		}

		public void Update()
		{
			string path = $"{outputDirectory}\\polar_coords_{crack.ID}_t{iteration}.vtk";

			List<double[]> pointsGlobal = GeneratePointsGlobalCartesian();
			List<XPoint> pointsNatural = GeneratePointsNatural(pointsGlobal);
			(List<double> r, List<double> theta) = CalcPolarCoords(pointsNatural);

			using (var writer = new VtkPointWriter(path))
			{
				writer.WritePoints(pointsGlobal, true);
				writer.WriteScalarField("r", r);
				writer.WriteScalarField("theta", theta);
			}

			++iteration;
		}

		private (List<double> r, List<double> theta) CalcPolarCoords(List<XPoint> pointsNatural)
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

		private List<XPoint> GeneratePointsNatural(List<double[]> pointsGlobal)
		{
			var pointsNatural = new List<XPoint>(pointsGlobal.Count);
			foreach (double[] globalCoords in pointsGlobal)
			{
				(int elementID, double[] naturalCoords) = mesh.FindElementContaining(globalCoords);

				var point = new XPoint(mesh.Dimension);
				point.Element = model.Elements[elementID];
				point.Coordinates[CoordinateSystem.GlobalCartesian] = globalCoords;
				point.Coordinates[CoordinateSystem.ElementNatural] = naturalCoords;

				pointsNatural.Add(point);
			}

			return pointsNatural;
		}

		private List<double[]> GeneratePointsGlobalCartesian()
		{
			return (mesh.Dimension == 2) ? GeneratePointsGlobalCartesian2D() : GeneratePointsGlobalCartesian3D();
		}

		private List<double[]> GeneratePointsGlobalCartesian2D()
		{
			var points = new List<double[]>(numPointsPerAxis * numPointsPerAxis);
			double tol = 1E-2 * (mesh.MaxCoordinates[0] - mesh.MinCoordinates[0]);
			double dx = (mesh.MaxCoordinates[0] - mesh.MinCoordinates[0] - 2 * tol) / (numPointsPerAxis - 1);
			double dy = (mesh.MaxCoordinates[1] - mesh.MinCoordinates[1] - 2 * tol) / (numPointsPerAxis - 1);
			for (int i = 0; i < numPointsPerAxis; ++i)
			{
				double x = mesh.MinCoordinates[0] + tol + i * dx;
				for (int j = 0; j < numPointsPerAxis; ++j)
				{
					double y = mesh.MinCoordinates[1] + tol + j * dy;
					double[] point = { x, y };
					LimitPointInsideBounds(point);
					points.Add(point);
				}
			}
			return points;
		}

		private List<double[]> GeneratePointsGlobalCartesian3D()
		{
			var points = new List<double[]>(numPointsPerAxis * numPointsPerAxis * numPointsPerAxis);
			double tol = 1E-2 * (mesh.MaxCoordinates[0] - mesh.MinCoordinates[0]);
			double dx = (mesh.MaxCoordinates[0] - mesh.MinCoordinates[0] - 2 * tol) / (numPointsPerAxis - 1);
			double dy = (mesh.MaxCoordinates[1] - mesh.MinCoordinates[1] - 2 * tol) / (numPointsPerAxis - 1);
			double dz = (mesh.MaxCoordinates[2] - mesh.MinCoordinates[2] - 2 * tol) / (numPointsPerAxis - 1);
			for (int i = 0; i < numPointsPerAxis; ++i)
			{
				double x = mesh.MinCoordinates[0] + tol + i * dx;
				for (int j = 0; j < numPointsPerAxis; ++j)
				{
					double y = mesh.MinCoordinates[1] + tol + j * dy;
					for (int k = 0; k < numPointsPerAxis; ++k)
					{
						double z = mesh.MinCoordinates[2] + tol + k * dz;
						double[] point = { x, y, z };
						LimitPointInsideBounds(point);
						points.Add(point);
					}
				}
			}
			return points;
		}

		private void LimitPointInsideBounds(double[] point)
		{
			for (int d = 0; d < point.Length; ++d)
			{
				if (point[d] < mesh.MinCoordinates[d])
				{
					point[d] = mesh.MinCoordinates[d];
				}
				if (point[d] > mesh.MaxCoordinates[d])
				{
					point[d] = mesh.MaxCoordinates[d];
				}
			}
		}

	}
}
