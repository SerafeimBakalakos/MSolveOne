namespace MGroup.XFEM.IsoXFEM.IsoXfemElements
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.Elements;
	using MGroup.XFEM.Entities;

	public interface IIsoXfemElement: IXFiniteElement
	{
		public enum Phase
		{
			solidElement,
			voidElement,
			boundaryElement
		}

		public int[] IdOnAxis { get; set; }
		public double SizeOfElement { get; set; }
		public Matrix StiffnessOfElement { get; set; }
		public Phase PhaseElement { get; set; }
		public Vector ElementLevelSet { get; set; }
		public double Thickness { get;  }
		public void DefinePhaseOfElement();
		public void SetIdOnAxis(params int[] idOnAxis);
	}
}
