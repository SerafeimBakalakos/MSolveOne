namespace MGroup.XFEM.IsoXFEM.SolidRatioComputations
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.Elements;

	public interface ISolidRatio
	{
		public Vector InitialSizeOfElements { get; }
		public Vector RelativeCriteria { get; set; }

		public Vector CalculateSolidRatio();
	}
}
