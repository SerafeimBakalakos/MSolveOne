namespace MGroup.XFEM.IsoXFEM.Tests
{
	using System;
	using System.Collections.Generic;
	using System.Diagnostics;
	using System.Text;

	using Xunit;

	public class ArraysMethodsTests
	{
		[Fact]
		public static void SetDiffTest()
		{
			int[] setA = { 0, 1, 2, 3, 4, 5, 6 };
			int[] setB = { 0, 2, 3, 6 };
			int[] diffExpected = { 1, 4, 5 };
			int[] diffComputed = ArraysMethods.SetDiff(setB, setA);
			Debug.Write(diffComputed.Length);
			Assert.Equal(diffExpected.Length, diffComputed.Length);
			for (int i = 0; i < diffExpected.Length; i++)
			{
				Assert.Equal(diffExpected[i], diffComputed[i]);
			}
		}

		[Fact]

		public static void UniqueTest()
		{
			int[] aDuplicate = { 0, 1, 1, 3, 4, 5, 4, 7 };
			int[] aUniqueExpected = { 0, 1, 3, 5, 4, 7 };
			int[] aUniqueComputed = ArraysMethods.Unique(aDuplicate);
			Debug.Write(aUniqueComputed.Length);
			Assert.Equal(aUniqueExpected.Length, aUniqueComputed.Length);
			for (int i = 0; i < aUniqueExpected.Length; i++)
			{
				Assert.Equal(aUniqueExpected[i], aUniqueComputed[i]);
			}
		}
}
}
