using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public interface ICrackFront2D
	{
		List<int> ActiveTips { get; }

		//TODO: Perhaps this should be stored in the vertices.
		List<CrackFrontSystem2D> CoordinateSystems { get; }

		List<Vertex2D> Vertices { get; }

		void UpdateGeometry(CrackFrontPropagation frontPropagation);
	}
}
