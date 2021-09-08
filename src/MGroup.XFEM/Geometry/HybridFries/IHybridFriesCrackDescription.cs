using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public interface IHybridFriesCrackDescription : IImplicitCrackGeometry
	{
		int ID { get; }

		double[] GetTripleLevelSetsOf(XNode node);

		public double[] InterpolateTripleLevelSets(XPoint point)
		{
			IReadOnlyList<XNode> nodes = point.Element.Nodes;
			var pointLevelSets = new double[3];
			for (int n = 0; n < nodes.Count; ++n)
			{
				double[] nodalLevelSets = GetTripleLevelSetsOf(nodes[n]);
				pointLevelSets[0] += point.ShapeFunctions[n] * nodalLevelSets[0];
				pointLevelSets[1] += point.ShapeFunctions[n] * nodalLevelSets[1];
				pointLevelSets[2] += point.ShapeFunctions[n] * nodalLevelSets[2];
			}
			return pointLevelSets;
		}
	}
}
