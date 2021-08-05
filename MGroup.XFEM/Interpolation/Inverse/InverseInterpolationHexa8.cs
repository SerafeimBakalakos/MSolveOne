using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;

namespace MGroup.XFEM.Interpolation.Inverse
{
	/// <summary>
	/// Inverse mapping of the isoparametric interpolation of a hexahedral finite element with 8 nodes. Since the original
	/// mapping is linear, there are analytic formulas, which are presented in 
	/// "The inverse mapping and distortion measures for 8-node hexahedral isoparametric elements", K. -Y. YuanY. -S. HuangH. -T. YangT. H. H. Pian, 1994
	/// https://link.springer.com/article/10.1007/BF00350284
	/// </summary>
	public class InverseInterpolationHexa8: IInverseInterpolation
	{
		public InverseInterpolationHexa8(IReadOnlyList<INode> nodes)
		{
			
		}

		public double[] TransformPointCartesianToNatural(double[] point)
		{
			throw new NotImplementedException();
		}
	}
}
