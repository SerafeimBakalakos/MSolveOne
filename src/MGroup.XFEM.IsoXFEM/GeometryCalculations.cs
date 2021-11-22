using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM
{
    /// <summary>
    /// A static class which contains methods for geometry calculation and applications.
    /// </summary>
    public static class GeometryCalculations
    {
        /// <summary>
        /// Calculate the area of a convex polygon.
        /// </summary>
        /// <param name="elementCoordinates"> The coordinates of the angle points of polygon. </param>
        /// <param name="n"> The number of angles.</param>
        /// <returns>The measure of area of the polygon.</returns>
        static public double Polygonarea(Matrix elementCoordinates, int n)
        {
            Vector X = elementCoordinates.GetColumn(0);
            Vector Y = elementCoordinates.GetColumn(1);
            double area = 0.0;
            int j = n - 1;
            for (int i = 0; i < n; i++)
            {
                area += (X[j] + X[i]) * (Y[j] - Y[i]);
                // j is previous vertex to i
                j = i;
            }
            // Return absolute value
            return Math.Abs(area / 2.0);
        }
        /// <summary>
        /// Calculate the mean coordinates (X and Y) of a polygon shape.
        /// </summary>
        /// <param name="subElementCoordinates">The coordinates of the angle points of polygon.</param>
        /// <returns>The coordinates X and Y of the mean point. </returns>
        static public Matrix Mean(Matrix subElementCoordinates)
        {
            int n = subElementCoordinates.NumRows;
            int k = subElementCoordinates.NumColumns;
            Matrix meanCoordinates = Matrix.CreateZero(1, k);
            for (int i = 0; i < k; i++)
            {
                double sumCoords = 0.00;
                for (int j = 0; j < n; j++)
                {
                    sumCoords = sumCoords + subElementCoordinates[j, i];
                }
                meanCoordinates[0, i] = sumCoords / n;
            }
            return meanCoordinates;

        }
    }
}
