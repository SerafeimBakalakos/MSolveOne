using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.Boundaries
{
	public class BoxDomainBoundary3D : IDomainBoundary3D
	{
		private readonly double[] minCoords;
		private readonly double[] maxCoords;
		private readonly double proximityTolerance;

		public BoxDomainBoundary3D(double[] minCoords, double[] maxCoords, double relativeTolerance = 1E-6)
		{
			this.minCoords = minCoords;
			this.maxCoords = maxCoords;

			double minSide = double.MaxValue;
			for (int d = 0; d < 3; ++d)
			{
				double side = maxCoords[d] - minCoords[d];
				if (side < minSide)
				{
					minSide = side;
				}
			}
			this.proximityTolerance = relativeTolerance * minSide;
		}

		/// <summary>
		/// The normal vector points outwards
		/// </summary>
		/// <param name="point">A point that must lie on the domain's boundary.</param>
		public double[] CalcNormalAtBoundaryPoint(double[] point)
		{
			BoxFace face = FindFaceOfPoint(point);
			if (face == BoxFace.MinX)
			{
				return new double[] { -1, 0, 0 };
			}
			else if (face == BoxFace.MaxX)
			{
				return new double[] { +1, 0, 0 };
			}
			else if (face == BoxFace.MinY)
			{
				return new double[] { 0, -1, 0 };
			}
			else if (face == BoxFace.MaxY)
			{
				return new double[] { 0, +1, 0 };
			}
			else if (face == BoxFace.MinZ)
			{
				return new double[] { 0, 0, -1 };
			}
			else if (face == BoxFace.MaxZ)
			{
				return new double[] { 0, 0, +1 };
			}
			else
			{
				throw new ArgumentException("The point provided does not lie on the boundary");
			}
		}

		public RelativePositionManifoldPoint FindRelativePositionOf(double[] point)
		{
			BoxFace face = FindFaceOfPoint(point);
			if (face != BoxFace.None)
			{
				return RelativePositionManifoldPoint.Boundary;
			}
			else
			{
				bool isInternal = (point[0] > minCoords[0]) && (point[0] < maxCoords[0]);
				isInternal &= (point[1] > minCoords[1]) && (point[1] < maxCoords[1]);
				isInternal &= (point[2] > minCoords[2]) && (point[2] < maxCoords[2]);
				return isInternal ? RelativePositionManifoldPoint.Internal : RelativePositionManifoldPoint.External;
			}
		}

		public void MinimizeOffsetOfBoundaryPoint(double[] point)
		{
			BoxFace face = FindFaceOfPoint(point);
			if (face == BoxFace.MinX)
			{
				point[0] = minCoords[0];
			}
			else if (face == BoxFace.MaxX)
			{
				point[0] = maxCoords[0];
			}
			else if (face == BoxFace.MinY)
			{
				point[1] = minCoords[1];
			}
			else if (face == BoxFace.MaxY)
			{
				point[1] = maxCoords[1];
			}
			else if (face == BoxFace.MinZ)
			{
				point[2] = minCoords[2];
			}
			else if (face == BoxFace.MaxZ)
			{
				point[2] = maxCoords[2];
			}
			else
			{
				throw new ArgumentException("The point provided does not lie on the boundary");
			}
		}

		public bool SurroundsPoint(double[] point)
		{
			bool isInside = (point[0] > minCoords[0]) && (point[0] < maxCoords[0]);
			isInside &= (point[1] > minCoords[1]) && (point[1] < maxCoords[1]);
			isInside &= (point[2] > minCoords[2]) && (point[2] < maxCoords[2]);
			return isInside;
		}

		/// <summary>
		/// 
		/// </summary>
		/// <param name="point"></param>
		/// <returns></returns>
		private BoxFace FindFaceOfPoint(double[] point)
		{
			if (Math.Abs(point[0] - minCoords[0]) < proximityTolerance)
			{
				return BoxFace.MinX;
			}
			else if (Math.Abs(point[0] - maxCoords[0]) < proximityTolerance)
			{
				return BoxFace.MaxX;
			}
			else if (Math.Abs(point[1] - minCoords[1]) < proximityTolerance)
			{
				return BoxFace.MinY;
			}
			else if (Math.Abs(point[1] - maxCoords[1]) < proximityTolerance)
			{
				return BoxFace.MaxY;
			}
			else if (Math.Abs(point[2] - minCoords[2]) < proximityTolerance)
			{
				return BoxFace.MinZ;
			}
			else if (Math.Abs(point[2] - maxCoords[2]) < proximityTolerance)
			{
				return BoxFace.MaxZ;
			}
			else
			{
				return BoxFace.None;
			}
		}

		private enum BoxFace
		{
			MinX, MaxX, MinY, MaxY, MinZ, MaxZ, None
		}
	}
}
