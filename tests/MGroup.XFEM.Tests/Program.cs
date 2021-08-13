using System;

namespace MGroup.XFEM.Tests
{
	class Program
	{
		static void Main(string[] args)
		{
			SpecialSolvers.MpiTestSuite.RunTestsWith4Processes();
			//Multiphase.EpoxyAg.Benchmark3D.RunHomogenization();
		}
	}
}
