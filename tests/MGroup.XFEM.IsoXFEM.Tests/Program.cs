using System;
using MGroup.XFEM.IsoXFEM;
using MGroup.XFEM.IsoXFEM.Solvers;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.IsoXFEM.Tests
{
	class Program
	{
		static void Main(string[] args)
		{
			ModelTests.CreateNodesTest();
			Console.WriteLine("Test Finished");
		}
	}
}
