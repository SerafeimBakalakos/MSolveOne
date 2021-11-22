using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM
{
    /// <summary>
    /// A static class which contains methods for computations and changes on arrays.
    /// </summary>
    public static class ArraysMethods
    {
        /// <summary>
        /// Finds the values of integer array b that are not included in integer array a.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns>integer array c with the values of integer array b that are not included in integer array a. </returns>
        public static int[] SetDiff(int[] a, int[] b)
        {           
            List<int> c = new List<int>();
            var diff = new bool[b.Length];
            for (int i = 0; i < b.Length; i++)
            {
                diff[i] = true;
            }
            for (int i = 0; i < a.Length; i++)
            {
                for (int j = 0; j < b.Length; j++)
                {
                    if (a[i] == b[j])
                    {
                        diff[j] = false;
                    }
                }
            }
            //int n = 0;
            for (int i = 0; i < diff.GetLength(0); i++)
            {
                if (diff[i] == true)
                {
                    c.Add(b[i]);                   
                }
            }
            int[] arrayResult = new int[c.Count];
            for (int i = 0; i < c.Count; i++)
            {
                arrayResult[i] = c[i];
            }
            return arrayResult;
        }

        /// <summary>
        /// Returns an integer arrays with all the values of integer array a but not the duplicates ones.
        /// </summary>
        /// <param name="a"></param>
        /// <returns>Returns an integer arrays with all the values of integer array a but not the duplicates ones.</returns>
        public static int[] Unique(int[] a)
        {
            // Use nested loop to check for duplicates.
            List<int> result = new List<int>();
            for (int i = 0; i < a.Length; i++)
            {
                // Check for duplicates in all following elements.
                bool isDuplicate = false;
                for (int y = i + 1; y < a.Length; y++)
                {
                    if (a[i] == a[y])
                    {
                        isDuplicate = true;
                        break;
                    }
                }
                if (!isDuplicate)
                {
                    result.Add(a[i]);
                }
            }
            int[] uniqueValues = new int[result.Count];
            for (int i = 0; i < result.Count; i++)
            {
                uniqueValues[i] = result[i];
            }
            return uniqueValues;
        }
    }
}
