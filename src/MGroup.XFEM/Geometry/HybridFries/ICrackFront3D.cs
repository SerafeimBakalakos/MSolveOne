using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public interface ICrackFront3D
	{
		List<int> ActiveTips { get; }

		//TODO: Perhaps this should be stored in the vertices.
		List<CrackTipSystem3D> CoordinateSystems { get; }

		List<Edge3D> Edges { get; }
		
		List<Vertex3D> Vertices { get; }

		PropagationMesh3D CreatePropagationMesh(CrackFrontPropagation frontPropagation);

		void Update();
	}
}
