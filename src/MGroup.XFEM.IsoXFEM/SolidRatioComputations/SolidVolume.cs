namespace MGroup.XFEM.IsoXFEM.SolidRatioComputations
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.IsoXFEM.IsoXfemElements;

	class SolidVolume : ISolidRatio
	{
		//public XModel<IsoXfemElement3D> ModelX { get; }
		public Vector InitialSizeOfElements { get; }
		public Vector RelativeCriteria { get; set; }
		public Vector CalculateSolidRatio() => throw new NotImplementedException();
	}
}
