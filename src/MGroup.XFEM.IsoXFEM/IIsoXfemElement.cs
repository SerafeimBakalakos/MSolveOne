namespace MGroup.XFEM.IsoXFEM
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.Elements;

	public interface IIsoXfemElement: IXFiniteElement
	{
		public enum Phase
		{
			solidElement,
			voidElement,
			boundaryElement
		}
		public double SizeOfElement { get; set; }
		public Matrix StiffnessOfElement { get; set; }
		public Phase PhaseElement { get; set; }
		public Vector ElementLevelSet { get; set; }
		public Matrix ElasticityMatrix { get; set; }
		public double Thickness { get;  }
		public void DefinePhaseOfElement();

	}
}
