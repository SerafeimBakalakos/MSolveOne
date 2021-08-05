using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Geometry.Coordinates;

namespace MGroup.XFEM.Geometry
{
    public class Point2DComparer : IComparer<double[]>
    {
        private readonly ValueComparer comparer;

        public Point2DComparer(double tolerance = 1E-4)
        {
            comparer = new ValueComparer(tolerance);
        }

        public int Compare(double[] point0, double[] point1)
        {
            if (comparer.AreEqual(point0[0], point1[0]) && comparer.AreEqual(point0[1], point1[1])) return 0;
            else if (point0[0] < point1[0]) return -1;
            else if (point0[0] > point1[0]) return +1;
            else
            {
                if (point0[1] < point1[1]) return -1;
                else if (point0[1] > point1[1]) return +1;
                else throw new Exception("Should not have happened");
            }
        }
    }

    public class Point2DComparer<TPoint> : IComparer<TPoint>
        where TPoint: IPoint
    {
        private readonly ValueComparer comparer;

        public Point2DComparer(double tolerance = 1E-4)
        {
            comparer = new ValueComparer(tolerance);
        }

        public int Compare(TPoint x, TPoint y)
        {
            if (comparer.AreEqual(x.X1, y.X1) && comparer.AreEqual(x.X2, y.X2)) return 0;
            else if (x.X1 < y.X1) return -1;
            else if (x.X1 > y.X1) return +1;
            else
            {
                if (x.X2 < y.X2) return -1;
                else if (x.X2 > y.X2) return +1;
                else throw new Exception("Should not have happened");
            }
        }
    }
}
