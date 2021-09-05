using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Geometry.LSM
{
	public interface IImplicitCrackGeometry
	{
		/// <summary>
		/// Returns the 2 level sets (phi, psi) stored at <paramref name="node"/>. 
		/// phi is the signed distance from the body (curve/surface) of the crack.
		/// psi is the signed distance from a curve/surface that is perpendicular to the crack body at the crack front.
		/// </summary>
		/// <param name="node"></param>
		/// <returns></returns>
		double[] GetNodalLevelSets(XNode node);

		public double[] InterpolateLevelSets(XPoint point)
		{
			IReadOnlyList<XNode> nodes = point.Element.Nodes;
			var pointLevelSets = new double[2];
			for (int n = 0; n < nodes.Count; ++n)
			{
				double[] nodalLevelSets = GetNodalLevelSets(nodes[n]);
				pointLevelSets[0] += point.ShapeFunctions[n] * nodalLevelSets[0];
				pointLevelSets[1] += point.ShapeFunctions[n] * nodalLevelSets[1];
			}
			return pointLevelSets;
		}
	}
}
