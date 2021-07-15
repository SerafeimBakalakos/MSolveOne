using System;
using MGroup.Solvers.DDM.Tests.PSM;

namespace MGroup.Solvers.DDM.Tests
{
    class Program
    {
        static void Main(string[] args)
        {
            MpiTestSuite.RunTestsWith4Processes();
        }
    }
}
