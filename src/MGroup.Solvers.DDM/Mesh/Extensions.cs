using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Solvers.DDM.Mesh
{
    public static class Extensions
    {
        public static int[] Copy(this int[] array)
        {
            var copy = new int[array.Length];
            Array.Copy(array, copy, array.Length);
            return copy;
        }

        public static double[] Copy(this double[] array)
        {
            var copy = new double[array.Length];
            Array.Copy(array, copy, array.Length);
            return copy;
        }
    }
}
