using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions
{
    public static class Utilities
    {
        public static bool AreIndices(IEnumerable<int> integers)
        {
            var sorted = new SortedSet<int>();
            foreach (int x in integers)
            {
                bool isUnique = sorted.Add(x);
                if (!isUnique)
                {
                    return false;
                }
            }

            int previous = -1;
            foreach (int x in sorted)
            {
                if (x != previous + 1)
                {
                    return false;
                }
                previous = x;
            }

            return true;
        }
    }
}
