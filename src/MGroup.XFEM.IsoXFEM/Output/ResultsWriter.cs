using MGroup.XFEM.IsoXFEM.Output;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Output;
using MGroup.LinearAlgebra.Output.Formatting;
using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM.Output
{
    class ResultsWriter
    {
        public static void ResultsWriterToTxt (Matrix results)
        {
            var writer = new FullMatrixWriter();
            writer.NumericFormat = new ExponentialFormat() { NumDecimalDigits = 17 };
            string path = $"{Paths.OutputForTxtResults}\\OOS_BottomEnd_40x20_InitialStiffness_ComputeOnlyOneTime_CorrectMatlabErrors.txt";
            writer.WriteToFile(results, path);
        }

    }
}
