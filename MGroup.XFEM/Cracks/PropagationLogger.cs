using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MGroup.XFEM.Cracks
{
    public class PropagationLogger
    {
        public List<double> SIFsMode1 { get; }
        public List<double> SIFsMode2 { get; }
        public List<double> GrowthAngles { get; }
        public List<double> GrowthLengths { get; }

        public PropagationLogger()
        {
            SIFsMode1 = new List<double>();
            SIFsMode2 = new List<double>();
            GrowthAngles = new List<double>();
            GrowthLengths = new List<double>();
        }

        public void PrintAnalysisStep(int iteration)
        {
            Console.WriteLine("Analysis step " + iteration + ":");
            Console.WriteLine("SIF - Mode 1 = " + SIFsMode1[iteration]);
            Console.WriteLine("SIF - Mode 2 = " + SIFsMode2[iteration]);
            Console.WriteLine("Crack growth angle (local tip system) = " + GrowthAngles[iteration]);
            Console.WriteLine("Crack growth length = " + GrowthLengths[iteration]);
        }
    }
}
