using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Output.Mesh
{
	public class UniformPointGenerator
	{
		private readonly XModel<IXCrackElement> model;
		private readonly ICartesianMesh mesh;
		private readonly int numPointsPerAxis;

		public UniformPointGenerator(XModel<IXCrackElement> model, ICartesianMesh mesh, int numPointsPerAxis)
		{
			this.model = model;
			this.mesh = mesh;
			this.numPointsPerAxis = numPointsPerAxis;
		}

		public List<double[]> GeneratePointsGlobalCartesian()
		{
			return (mesh.Dimension == 2) ? GeneratePointsGlobalCartesian2D() : GeneratePointsGlobalCartesian3D();
		}

		public List<XPoint> GeneratePointsNatural(List<double[]> pointsGlobal)
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
