using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Solvers.DDM.Mesh
{
    public static class Utilities
    {
        public static bool AreContiguousUniqueIndices(int[] numbers)
        {
            int[] copy = numbers.Copy();
            Array.Sort(copy);

            if (copy[0] != 0) return false;
            for (int i = 0; i < copy.Length - 1; ++i)
            {
                if (copy[i + 1] != copy[i] + 1) return false;
            }

            return true;
        }
    }
}
