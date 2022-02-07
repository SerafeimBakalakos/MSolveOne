using MGroup.XFEM.IsoXFEM.Output;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Output;
using MGroup.LinearAlgebra.Output.Formatting;
using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.IsoXFEM.IsoXfemElements;
using System.Linq;

namespace MGroup.XFEM.IsoXFEM.Output
{
    class ResultsWriter
    {
        public static void ResultsWriterToTxt (Matrix results)
        {
            var writer = new FullMatrixWriter();
            writer.NumericFormat = new ExponentialFormat() { NumDecimalDigits = 17 };
            string path = $"{Paths.OutputForTxtResults}\\3D_40x20x2_2.txt";
            writer.WriteToFile(results, path);
        }

		public static void VolumeForEachElementWriter(int iter,int dimension, Dictionary<int, IIsoXfemElement> Elements)
		{
			var writer = new FullMatrixWriter();
			writer.NumericFormat = new ExponentialFormat() { NumDecimalDigits = 4 };
			string path = $"{Paths.OutputElementsSize}\\ElementsSize_iter_0.txt";
			var b = iter;
			string step = b.ToString();
			var outputFile = path.Replace("0", step);
			var matrixWithCoordinatesAndVolume = Matrix.CreateZero(Elements.Count, Elements.Values.First().IdOnAxis.Length + 1);
			for (int i = 0; i < matrixWithCoordinatesAndVolume.NumRows; i++)
			{
				for (int j = 0; j < Elements[i].IdOnAxis.Length; j++)
				{
						matrixWithCoordinatesAndVolume[i, j] = Elements[i].IdOnAxis[j];
				}
				matrixWithCoordinatesAndVolume[i, matrixWithCoordinatesAndVolume.NumColumns - 1] = Elements[i].SizeOfElement;
			}
			writer.WriteToFile(matrixWithCoordinatesAndVolume, outputFile);
		}

    }
}
