using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.AssemblerExtensions;
using MGroup.Solvers.DDM.Commons;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemMatrixDense : IFetiDPCoarseProblemGlobalMatrix
	{
		private readonly DenseMatrixAssembler assembler = new DenseMatrixAssembler();
		private Matrix inverseSccGlobal;
		
		public void InvertGlobalScc(int numGlobalCornerDofs, IDictionary<int, int[]> subdomainToGlobalCornerDofs, 
			IDictionary<int, IMatrix>  subdomainMatricesScc)
		{
			Matrix globalScc = 
				assembler.BuildGlobalMatrix(numGlobalCornerDofs, subdomainToGlobalCornerDofs, subdomainMatricesScc);
			#region debug
			var writer = new LinearAlgebra.Output.FullMatrixWriter();
			writer.ArrayFormat = LinearAlgebra.Output.Formatting.Array2DFormat.CSharpArray;
			//writer.NumericFormat = new LinearAlgebra.Output.Formatting.ExponentialFormat() { NumDecimalDigits = 15 };
			writer.NumericFormat = new LinearAlgebra.Output.Formatting.GeneralNumericFormat() ;
			string file = @"C:\Users\Serafeim\Desktop\PFETIDP\matrices\SccDense.txt";
			writer.WriteToFile(globalScc, file);

			if (System.IO.File.Exists(@"C:\Users\Serafeim\Desktop\PFETIDP\matrices\yc.txt"))
			{
				System.IO.File.Delete(@"C:\Users\Serafeim\Desktop\PFETIDP\matrices\yc.txt");
			}
			if (System.IO.File.Exists(@"C:\Users\Serafeim\Desktop\PFETIDP\matrices\xc.txt"))
			{
				System.IO.File.Delete(@"C:\Users\Serafeim\Desktop\PFETIDP\matrices\xc.txt");
			}

			#endregion
			globalScc.InvertInPlace();
			inverseSccGlobal = globalScc;
		}

		public void MultiplyInverseScc(Vector input, Vector output)
		{
			inverseSccGlobal.MultiplyIntoResult(input, output);

			#region debug
			var writer = new LinearAlgebra.Output.FullVectorWriter();
			writer.ArrayFormat = LinearAlgebra.Output.Formatting.Array1DFormat.CSharpArray;
			writer.NumericFormat = new LinearAlgebra.Output.Formatting.GeneralNumericFormat();
			string fileRhs = @"C:\Users\Serafeim\Desktop\PFETIDP\matrices\yc.txt";
			string fileSol = @"C:\Users\Serafeim\Desktop\PFETIDP\matrices\xc.txt";
			writer.WriteToFile(input, fileRhs, true);
			writer.WriteToFile(output, fileSol, true);
			#endregion
		}

		public DofPermutation ReorderGlobalCornerDofs(int numGlobalCornerDofs, 
			IDictionary<int, int[]> subdomainToGlobalCornerDofs)
			=> DofPermutation.CreateNoPermutation();
	}
}
